param(
    [string]$Port = 'COM32',
    [ValidateRange(1, 4000000)][int]$BaudRate = 115200,
    [string]$OutputDir = (Join-Path $PSScriptRoot 'logs'),
    [string]$ReplayFile = ''
)

$ErrorActionPreference = 'Stop'
$names = @('target_mps', 'filtered_mps', 'speed_angle_deg', 'target_angle_deg', 'angle_deg', 'balance_pwm',
    'device_time_ms', 'raw_mps', 'pi_feedback_mps', 'error_mps', 'p_angle_deg', 'i_angle_deg', 'gyro_deg_s',
    'turn_request_pwm', 'turn_actual_pwm', 'left_count_2ms', 'right_count_2ms', 'left_pwm', 'right_pwm',
    'pi_active', 'running', 'fault')
$utf8 = New-Object System.Text.UTF8Encoding($false)
$sessionDir = Join-Path $OutputDir ((Get-Date -Format 'yyyyMMdd_HHmmss_fff') + '_' + ($Port -replace '[^a-zA-Z0-9_-]', '_'))
[void][IO.Directory]::CreateDirectory($sessionDir)
$rawPath = Join-Path $sessionDir 'serial.bin'
$textPath = Join-Path $sessionDir 'serial.txt'
$csvPath = Join-Path $sessionDir 'speed.csv'
$rawLog = $null
$textLog = $null
$csvLog = $null
$serial = $null
$inputFile = $null
$buffer = New-Object byte[] 8192
$pending = ''
$received = [long]0
$frames = [long]0
$malformed = [long]0
$activeFrames = [long]0
$runningFrames = [long]0
$lastState = 'No complete speed frame yet'
$lastStatus = [DateTime]::MinValue
$lastFrameTime = [DateTime]::MinValue
$lastFlush = [DateTime]::UtcNow
$nextOpen = [DateTime]::MinValue
$replay = $ReplayFile -ne ''

function Save-ReceivedBytes([int]$count) {
    $script:rawLog.Write($script:buffer, 0, $count)
    $text = [Text.Encoding]::ASCII.GetString($script:buffer, 0, $count)
    $script:textLog.Write($text)
    $script:received += $count
    $script:pending += $text

    # A read can split a frame anywhere, including in the prefix or the final number.
    while (($newline = $script:pending.IndexOf("`n")) -ge 0) {
        $line = $script:pending.Substring(0, $newline).TrimEnd("`r")
        $script:pending = $script:pending.Substring($newline + 1)
        if (!$line.StartsWith('speed:')) { continue }
        $fields = $line.Substring(6).Split(',')
        $valid = $fields.Count -eq 22 -or $fields.Count -eq 6
        foreach ($field in $fields) {
            if ($field -notmatch '^(?:[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?|[+-]?(?:nan|inf(?:inity)?))$') {
                $valid = $false
                break
            }
        }
        if (!$valid) {
            $script:malformed++
            continue
        }
        $row = @([DateTime]::UtcNow.ToString('o'), $fields.Count) + $fields
        if ($fields.Count -eq 6) { $row += @('') * 16 }
        $script:csvLog.WriteLine($row -join ',')
        $script:frames++
        $script:lastFrameTime = [DateTime]::UtcNow
        if ($fields.Count -eq 22) {
            if ($fields[19] -eq '1') { $script:activeFrames++ }
            if ($fields[20] -eq '1') { $script:runningFrames++ }
            $script:lastState = 'PI={0} RUN={1} FAULT={2} target={3} speed={4}' -f
                $fields[19], $fields[20], $fields[21], $fields[0], $fields[1]
        } else {
            $script:lastState = 'Old 6-column firmware: PI/RUN flags unavailable'
        }
    }
    # Raw bytes are still saved if binary image data or a damaged frame has no newline.
    if ($script:pending.Length -gt 65536) {
        $script:pending = ''
        $script:malformed++
    }
}

try {
    $rawLog = New-Object IO.FileStream($rawPath, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::Read)
    $textLog = New-Object IO.StreamWriter($textPath, $false, $utf8)
    $csvLog = New-Object IO.StreamWriter($csvPath, $false, $utf8)
    $csvLog.WriteLine((@('pc_time_iso', 'frame_columns') + $names) -join ',')
    $csvLog.Flush()
    Write-Host ('Logs: ' + $sessionDir)
    if ($replay) {
        $inputFile = [IO.File]::OpenRead((Resolve-Path -LiteralPath $ReplayFile).Path)
        while (($count = $inputFile.Read($buffer, 0, $buffer.Length)) -gt 0) { Save-ReceivedBytes $count }
        Write-Host 'Replay complete. PC timestamps are replay times; device timestamps are unchanged.'
    } else {
        Write-Host ('Recording {0}, {1} baud, 8N1. Close VOFA/serial assistants using this port.' -f $Port, $BaudRate)
        Write-Host 'Start recording BEFORE FOLLOW START. Press Q or Ctrl+C to finish.'
        while ($true) {
            if (![Console]::IsInputRedirected -and [Console]::KeyAvailable) {
                if ([Console]::ReadKey($true).Key -eq [ConsoleKey]::Q) { break }
            }
            if ($null -eq $serial) {
                if ([DateTime]::UtcNow -lt $nextOpen) {
                    Start-Sleep -Milliseconds 100
                    continue
                }
                $serial = New-Object IO.Ports.SerialPort($Port, $BaudRate, [IO.Ports.Parity]::None, 8, [IO.Ports.StopBits]::One)
                $serial.ReadTimeout = 100
                $serial.ReadBufferSize = 65536
                $serial.Handshake = [IO.Ports.Handshake]::None
                $serial.DtrEnable = $false
                $serial.RtsEnable = $false
                try {
                    $serial.Open()
                    Write-Host ('Connected: ' + $Port)
                } catch {
                    Write-Host ('Cannot open {0}: {1}. Retrying in 3 seconds; Q to stop.' -f $Port, $_.Exception.Message)
                    $serial.Dispose()
                    $serial = $null
                    $nextOpen = [DateTime]::UtcNow.AddSeconds(3)
                    continue
                }
            }
            $count = 0
            try {
                $count = $serial.Read($buffer, 0, $buffer.Length)
            } catch [TimeoutException] {
                # An idle UART is normal; a timeout must not end the recording.
            } catch {
                Write-Host ('Serial read failed: {0}. Reconnecting in 3 seconds.' -f $_.Exception.Message)
                $serial.Dispose()
                $serial = $null
                $pending = ''
                $lastState = 'Disconnected; waiting for a new complete frame'
                $nextOpen = [DateTime]::UtcNow.AddSeconds(3)
            }
            # Disk errors must end with an error, rather than being mistaken for UART disconnects.
            if ($count -gt 0) { Save-ReceivedBytes $count }
            $now = [DateTime]::UtcNow
            if (($now - $lastFlush).TotalSeconds -ge 1) {
                $rawLog.Flush()
                $textLog.Flush()
                $csvLog.Flush()
                $lastFlush = $now
            }
            if (($now - $lastStatus).TotalSeconds -ge 1) {
                $age = 'never'
                if ($lastFrameTime -ne [DateTime]::MinValue) { $age = '{0:F1}s' -f ($now - $lastFrameTime).TotalSeconds }
                Write-Host ('Bytes={0} frames={1} PI-ON frames={2} frame-age={3} | {4}' -f
                    $received, $frames, $activeFrames, $age, $lastState)
                $lastStatus = $now
            }
        }
    }
} finally {
    if ($null -ne $serial) { $serial.Dispose() }
    if ($null -ne $inputFile) { $inputFile.Dispose() }
    if ($null -ne $rawLog) { $rawLog.Dispose() }
    if ($null -ne $textLog) { $textLog.Dispose() }
    if ($null -ne $csvLog) { $csvLog.Dispose() }
    Write-Host ('Saved {0} bytes, {1} frames, {2} RUN frames, {3} PI-ON frames, {4} malformed frames.' -f
        $received, $frames, $runningFrames, $activeFrames, $malformed)
    Write-Host ('Text: ' + $textPath)
    Write-Host ('CSV:  ' + $csvPath)
    Write-Host ('Raw:  ' + $rawPath)
    if (!$replay -and $activeFrames -eq 0) { Write-Host 'WARNING: No frame with PI=1 was recorded.' }
}
