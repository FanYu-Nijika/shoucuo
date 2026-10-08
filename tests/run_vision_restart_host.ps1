$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $projectRoot
$outputDirectory = Join-Path $workspaceRoot 'tmp\stop_restart_tests'
$binary = Join-Path $outputDirectory 'vision_restart_host.exe'
$commonHeaders = Join-Path $projectRoot 'libraries\zf_common'
$deviceHeaders = Join-Path $projectRoot 'libraries\zf_device'
$compiler = Get-Command gcc -ErrorAction Stop

New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null

$arguments = @(
    '-std=c99',
    '-Wall',
    '-Wextra',
    '-Wno-unused-parameter',
    '-include', (Join-Path $PSScriptRoot 'vision_host_stubs.h'),
    '-I', (Join-Path $projectRoot 'code'),
    '-I', $commonHeaders,
    '-I', $deviceHeaders,
    (Join-Path $projectRoot 'code\image.c'),
    (Join-Path $projectRoot 'code\longest_white.c'),
    (Join-Path $PSScriptRoot 'vision_restart_host.c'),
    '-lm',
    '-o', $binary
)

& $compiler.Source @arguments
if ($LASTEXITCODE -ne 0) { throw "Host regression compile failed with exit code $LASTEXITCODE" }

& $binary
if ($LASTEXITCODE -ne 0) { throw "Host regression failed with exit code $LASTEXITCODE" }
