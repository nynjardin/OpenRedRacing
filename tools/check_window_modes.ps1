[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Executable,
    [Parameter(Mandatory)][string]$GameDirectory,
    [Parameter(Mandatory)][string]$OutputDirectory,
    [switch]$IncludeMenu,
    [switch]$IncludeRuf1Jcb,
    [ValidateRange(1, 300)][int]$WindowTimeoutSeconds = 45,
    [ValidateRange(0, 64)][int]$RestoreBoundsTolerancePixels = 4
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

if (-not [Environment]::Is64BitProcess -or -not [Environment]::Is64BitOperatingSystem) {
    throw 'Run this probe with 64-bit PowerShell on Windows x64.'
}

function Resolve-GameDirectory([string]$Path) {
    $item = Get-Item -LiteralPath $Path -ErrorAction Stop
    if (-not $item.PSIsContainer) { throw "GameDirectory is not a directory: $($item.FullName)" }
    if (Test-Path -LiteralPath (Join-Path $item.FullName 'ALWAYS.WAP') -PathType Leaf) {
        return $item.FullName
    }
    $nested = Join-Path $item.FullName 'BIGRED'
    if (Test-Path -LiteralPath (Join-Path $nested 'ALWAYS.WAP') -PathType Leaf) {
        return $nested
    }
    throw "GameDirectory has no ALWAYS.WAP (checked it and its BIGRED child): $($item.FullName)"
}

if (-not ('OpenRedWindowProbeNative' -as [type])) {
    Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

public static class OpenRedWindowProbeNative
{
    private delegate bool EnumWindowsProc(IntPtr hwnd, IntPtr lParam);

    [StructLayout(LayoutKind.Sequential)]
    public struct Rect { public int Left, Top, Right, Bottom; }

    [StructLayout(LayoutKind.Sequential)]
    public struct MonitorInfo {
        public int cbSize;
        public Rect rcMonitor;
        public Rect rcWork;
        public uint dwFlags;
    }

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool EnumWindows(EnumWindowsProc callback, IntPtr lParam);

    [DllImport("user32.dll")]
    private static extern uint GetWindowThreadProcessId(IntPtr hwnd, out uint processId);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    private static extern int GetClassName(IntPtr hwnd, StringBuilder text, int count);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    private static extern int GetWindowText(IntPtr hwnd, StringBuilder text, int count);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool IsWindow(IntPtr hwnd);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool IsWindowVisible(IntPtr hwnd);

    [DllImport("user32.dll")]
    public static extern bool ShowWindowAsync(IntPtr hwnd, int command);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool GetClientRect(IntPtr hwnd, out Rect rect);

    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool GetWindowRect(IntPtr hwnd, out Rect rect);

    [DllImport("user32.dll", EntryPoint = "GetWindowLongPtrW")]
    private static extern IntPtr GetWindowLongPtr(IntPtr hwnd, int index);

    [DllImport("user32.dll")]
    private static extern IntPtr MonitorFromWindow(IntPtr hwnd, uint flags);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool GetMonitorInfo(IntPtr monitor, ref MonitorInfo info);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool PostMessage(IntPtr hwnd, uint message, IntPtr wParam, IntPtr lParam);

    [DllImport("user32.dll")]
    public static extern uint MapVirtualKey(uint code, uint mapType);

    [DllImport("user32.dll")]
    public static extern IntPtr SetThreadDpiAwarenessContext(IntPtr context);

    public static IntPtr[] GetTopLevelWindows(int processId) {
        var result = new List<IntPtr>();
        EnumWindows(delegate(IntPtr hwnd, IntPtr unused) {
            uint owner;
            GetWindowThreadProcessId(hwnd, out owner);
            if (owner == (uint)processId) result.Add(hwnd);
            return true;
        }, IntPtr.Zero);
        return result.ToArray();
    }

    public static string GetClass(IntPtr hwnd) {
        var text = new StringBuilder(256);
        GetClassName(hwnd, text, text.Capacity);
        return text.ToString();
    }

    public static string GetTitle(IntPtr hwnd) {
        var text = new StringBuilder(1024);
        GetWindowText(hwnd, text, text.Capacity);
        return text.ToString();
    }

    public static uint GetStyle(IntPtr hwnd) {
        return unchecked((uint)GetWindowLongPtr(hwnd, -16).ToInt64());
    }

    public static Rect GetMonitorRect(IntPtr hwnd) {
        IntPtr monitor = MonitorFromWindow(hwnd, 2u);
        var info = new MonitorInfo();
        info.cbSize = Marshal.SizeOf(typeof(MonitorInfo));
        if (monitor == IntPtr.Zero || !GetMonitorInfo(monitor, ref info)) {
            throw new InvalidOperationException("GetMonitorInfo failed");
        }
        return info.rcMonitor;
    }
}
'@
}

[void][OpenRedWindowProbeNative]::SetThreadDpiAwarenessContext([IntPtr](-4))

$exe = (Get-Item -LiteralPath $Executable -ErrorAction Stop).FullName
if ((Get-Item -LiteralPath $exe).PSIsContainer) { throw "Executable is a directory: $exe" }
$game = Resolve-GameDirectory $GameDirectory
$runRoot = [IO.Path]::GetFullPath($OutputDirectory)
$runDir = Join-Path $runRoot (Get-Date -Format 'yyyyMMdd-HHmmss-fff')
New-Item -ItemType Directory -Path $runDir -Force | Out-Null

$createdProcesses = [Collections.Generic.List[System.Diagnostics.Process]]::new()
$createdPids = [Collections.Generic.List[int]]::new()
$results = [Collections.Generic.List[object]]::new()
$caseNumber = 0

function ConvertTo-NativeArg([AllowEmptyString()][string]$Value) {
    if ($Value.Length -eq 0) { return '""' }
    if ($Value -notmatch '[\s"]') { return $Value }
    $builder = [Text.StringBuilder]::new('"')
    $slashes = 0
    foreach ($character in $Value.ToCharArray()) {
        if ($character -eq '\') { ++$slashes; continue }
        if ($character -eq '"') {
            [void]$builder.Append(('\' * (($slashes * 2) + 1))).Append('"')
            $slashes = 0
            continue
        }
        if ($slashes) { [void]$builder.Append('\' * $slashes); $slashes = 0 }
        [void]$builder.Append($character)
    }
    if ($slashes) { [void]$builder.Append('\' * ($slashes * 2)) }
    [void]$builder.Append('"')
    return $builder.ToString()
}

function Join-NativeArgs([string[]]$Arguments) {
    (($Arguments | ForEach-Object { ConvertTo-NativeArg ([string]$_) }) -join ' ')
}

function Find-SdlWindow([int]$ProcessId, [switch]$IncludeHidden) {
    $fallback = [IntPtr]::Zero
    foreach ($handle in [OpenRedWindowProbeNative]::GetTopLevelWindows($ProcessId)) {
        $class = [OpenRedWindowProbeNative]::GetClass($handle)
        $title = [OpenRedWindowProbeNative]::GetTitle($handle)
        if ($class -ne 'SDL_app') { continue }
        if ([OpenRedWindowProbeNative]::IsWindowVisible($handle)) { return $handle }
        if ($fallback -eq [IntPtr]::Zero) { $fallback = $handle }
    }
    if ($IncludeHidden) { return $fallback }
    [IntPtr]::Zero
}

function Get-WindowSnapshot([IntPtr]$Handle, [int]$ProcessId) {
    if ($Handle -eq [IntPtr]::Zero -or -not [OpenRedWindowProbeNative]::IsWindow($Handle)) {
        throw "Invalid SDL window for pid $ProcessId"
    }
    $client = [OpenRedWindowProbeNative+Rect]::new()
    $window = [OpenRedWindowProbeNative+Rect]::new()
    if (-not [OpenRedWindowProbeNative]::GetClientRect($Handle, [ref]$client)) { throw 'GetClientRect failed' }
    if (-not [OpenRedWindowProbeNative]::GetWindowRect($Handle, [ref]$window)) { throw 'GetWindowRect failed' }
    $monitor = [OpenRedWindowProbeNative]::GetMonitorRect($Handle)
    $style = [OpenRedWindowProbeNative]::GetStyle($Handle)
    [pscustomobject][ordered]@{
        capturedAtUtc = [DateTime]::UtcNow.ToString('o')
        processId = $ProcessId
        hwnd = ('0x{0:X}' -f $Handle.ToInt64())
        className = [OpenRedWindowProbeNative]::GetClass($Handle)
        title = [OpenRedWindowProbeNative]::GetTitle($Handle)
        visible = [OpenRedWindowProbeNative]::IsWindowVisible($Handle)
        clientWidth = $client.Right - $client.Left
        clientHeight = $client.Bottom - $client.Top
        windowLeft = $window.Left
        windowTop = $window.Top
        windowRight = $window.Right
        windowBottom = $window.Bottom
        monitorWidth = $monitor.Right - $monitor.Left
        monitorHeight = $monitor.Bottom - $monitor.Top
        style = ('0x{0:X8}' -f $style)
        hasCaption = (($style -band 0x00C00000) -ne 0)
    }
}

function Wait-Window([Diagnostics.Process]$Process, [scriptblock]$Predicate, [string]$Description) {
    $watch = [Diagnostics.Stopwatch]::StartNew()
    $last = $null
    while ($watch.Elapsed.TotalSeconds -lt $WindowTimeoutSeconds) {
        $Process.Refresh()
        if ($Process.HasExited) {
            throw "pid $($Process.Id) exited with code $($Process.ExitCode) while waiting for $Description"
        }
        $handle = Find-SdlWindow $Process.Id -IncludeHidden
        if ($handle -ne [IntPtr]::Zero) {
            # STARTF_USESHOWWINDOW from Start-Process -WindowStyle Hidden can
            # suppress SDL's first show. Reveal only this test window without
            # activating it; no foreground change or global input is needed.
            if (-not [OpenRedWindowProbeNative]::IsWindowVisible($handle)) {
                [void][OpenRedWindowProbeNative]::ShowWindowAsync($handle, 4)
            }
            # SDL can create and destroy a temporary GL window during startup.
            # A handle found by EnumWindows may disappear before the snapshot.
            try {
                $last = Get-WindowSnapshot $handle $Process.Id
                if (& $Predicate $last) { return $last }
            } catch {
                if ([OpenRedWindowProbeNative]::IsWindow($handle)) { throw }
            }
        }
        Start-Sleep -Milliseconds 150
    }
    $detail = if ($last) { $last | ConvertTo-Json -Compress } else { 'window not found' }
    throw "Timed out waiting for ${Description}: $detail"
}

function Assert-Condition([bool]$Condition, [string]$Message) {
    if (-not $Condition) { throw $Message }
}

function Get-KeyLParam([int]$VirtualKey, [bool]$Down) {
    $scan = [int]([OpenRedWindowProbeNative]::MapVirtualKey([uint32]$VirtualKey, 0) -band 0xFF)
    [int64]$value = 1 -bor ([int64]$scan -shl 16) -bor ([int64]1 -shl 29)
    if (-not $Down) { $value = $value -bor ([int64]1 -shl 30) -bor ([int64]1 -shl 31) }
    [IntPtr]$value
}

function Send-TargetedAltEnter([IntPtr]$Handle) {
    # WM_SYSKEYDOWN makes SDL report Return with KMOD_ALT. This targets one HWND;
    # it never changes foreground focus or calls SendInput/keybd_event.
    $altDown = [OpenRedWindowProbeNative]::PostMessage(
        $Handle, 0x0104, [IntPtr]0x12, (Get-KeyLParam 0x12 $true))
    try {
        Assert-Condition $altDown 'PostMessage(WM_SYSKEYDOWN, Alt) failed'
        Start-Sleep -Milliseconds 40
        $returnDown = [OpenRedWindowProbeNative]::PostMessage(
            $Handle, 0x0104, [IntPtr]0x0D, (Get-KeyLParam 0x0D $true))
        Assert-Condition $returnDown 'PostMessage(WM_SYSKEYDOWN, Enter) failed'
    } finally {
        [void][OpenRedWindowProbeNative]::PostMessage(
            $Handle, 0x0105, [IntPtr]0x0D, (Get-KeyLParam 0x0D $false))
        if ($altDown) {
            [void][OpenRedWindowProbeNative]::PostMessage(
                $Handle, 0x0105, [IntPtr]0x12, (Get-KeyLParam 0x12 $false))
        }
    }
}

function Start-ProbeProcess([string]$CaseDir, [string[]]$Arguments) {
    $stdout = Join-Path $CaseDir 'stdout.log'
    $stderr = Join-Path $CaseDir 'stderr.log'
    $process = Start-Process -FilePath $exe -ArgumentList (Join-NativeArgs $Arguments) `
        -WorkingDirectory (Split-Path -Parent $exe) -WindowStyle Hidden `
        -RedirectStandardOutput $stdout -RedirectStandardError $stderr -PassThru
    if (-not $process) { throw 'Start-Process returned no process' }
    $createdProcesses.Add($process)
    $createdPids.Add([int]$process.Id)
    [pscustomobject]@{ Process = $process; Pid = [int]$process.Id; Stdout = $stdout; Stderr = $stderr }
}

function Stop-CreatedProcess($Launch) {
    $cleanup = [ordered]@{ gracefulExit = $false; forcedTermination = $false; exitCode = $null; error = $null }
    if (-not $Launch) { return [pscustomobject]$cleanup }
    $process = $Launch.Process
    try {
        $process.Refresh()
        if (-not $process.HasExited) {
            $handle = Find-SdlWindow $process.Id -IncludeHidden
            if ($handle -ne [IntPtr]::Zero) {
                [void][OpenRedWindowProbeNative]::PostMessage($handle, 0x0010, [IntPtr]::Zero, [IntPtr]::Zero)
            }
            if (-not $process.WaitForExit(5000)) {
                Stop-Process -Id $process.Id -Force -ErrorAction Stop
                $cleanup.forcedTermination = $true
                [void]$process.WaitForExit(5000)
            }
        }
        $process.Refresh()
        if ($process.HasExited) {
            $cleanup.gracefulExit = -not $cleanup.forcedTermination
            $cleanup.exitCode = $process.ExitCode
        }
    } catch {
        $cleanup.error = $_.Exception.Message
        try {
            $process.Refresh()
            if (-not $process.HasExited) {
                Stop-Process -Id $process.Id -Force -ErrorAction Stop
                $cleanup.forcedTermination = $true
            }
        } catch {
            $cleanup.error += "; targeted termination also failed: $($_.Exception.Message)"
        }
    }
    [pscustomobject]$cleanup
}

function New-Case([string]$Name, [string[]]$Arguments) {
    ++$caseNumber
    $dir = Join-Path $runDir ('{0:D2}-{1}' -f $caseNumber, $Name)
    New-Item -ItemType Directory -Path $dir -Force | Out-Null
    [pscustomobject][ordered]@{
        name = $Name
        status = 'RUNNING'
        directory = $dir
        arguments = $Arguments
        commandLine = Join-NativeArgs (@($exe) + $Arguments)
        snapshots = @()
        assertions = @()
        cleanup = $null
        error = $null
    }
}

function Complete-Case($Case, $Launch, [string]$Failure) {
    $Case.cleanup = Stop-CreatedProcess $Launch
    if ($Failure) {
        $Case.status = 'FAIL'
        $Case.error = $Failure
    } elseif ($Case.cleanup.error) {
        $Case.status = 'FAIL'
        $Case.error = "Cleanup failed: $($Case.cleanup.error)"
    } else {
        $Case.status = 'PASS'
    }
    $Case | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $Case.directory 'result.json') -Encoding utf8
    ConvertTo-Json -InputObject @($Case.snapshots) -Depth 10 |
        Set-Content -LiteralPath (Join-Path $Case.directory 'window-snapshots.json') -Encoding utf8
    $results.Add([pscustomobject]$Case)
}

function Test-RoundTrip([string]$Name, [string[]]$BaseArguments) {
    $arguments = @($BaseArguments) + @('-WINDOWED', '-SIZE=960x540')
    $case = New-Case $Name $arguments
    $launch = $null
    $failure = $null
    try {
        $launch = Start-ProbeProcess $case.directory $arguments
        $initial = Wait-Window $launch.Process {
            param($window)
            $window.visible -and $window.clientWidth -eq 960 -and
            $window.clientHeight -eq 540 -and $window.hasCaption
        } 'visible captioned 960x540 startup'
        $case.snapshots += $initial
        Assert-Condition ($initial.className -eq 'SDL_app') "Window class is '$($initial.className)', expected SDL_app"
        $original = @($initial.windowLeft, $initial.windowTop, $initial.windowRight, $initial.windowBottom)
        $monitorWidth = $initial.monitorWidth
        $monitorHeight = $initial.monitorHeight

        Send-TargetedAltEnter (Find-SdlWindow $launch.Pid)
        $fullscreen = Wait-Window $launch.Process {
            param($window)
            $window.visible -and -not $window.hasCaption -and
            $window.clientWidth -eq $monitorWidth -and $window.clientHeight -eq $monitorHeight
        } 'Alt+Enter borderless fullscreen'
        $case.snapshots += $fullscreen

        Send-TargetedAltEnter (Find-SdlWindow $launch.Pid)
        $restored = Wait-Window $launch.Process {
            param($window)
            $window.visible -and $window.clientWidth -eq 960 -and
            $window.clientHeight -eq 540 -and $window.hasCaption
        } 'Alt+Enter restored window'
        $case.snapshots += $restored

        $restoredBounds = @($restored.windowLeft, $restored.windowTop, $restored.windowRight, $restored.windowBottom)
        $delta = 0
        for ($i = 0; $i -lt 4; ++$i) {
            $delta = [Math]::Max($delta, [Math]::Abs([int]$restoredBounds[$i] - [int]$original[$i]))
        }
        $case | Add-Member -NotePropertyName restoredBoundsDeltaPixels -NotePropertyValue $delta -Force
        Assert-Condition ($delta -le $RestoreBoundsTolerancePixels) `
            "Restored outer bounds moved $delta pixels; tolerance is $RestoreBoundsTolerancePixels"
        $case.assertions = @(
            'startup client is 960x540',
            'startup style has WS_CAPTION',
            'targeted Alt+Enter reaches borderless monitor size with no caption',
            'second targeted Alt+Enter restores client and outer bounds'
        )
    } catch {
        $failure = $_.Exception.Message
    } finally {
        Complete-Case $case $launch $failure
    }
}

function Test-BorderlessStartup([string]$Name, [string[]]$BaseArguments) {
    $arguments = @($BaseArguments) + @('-BORDERLESS', '-RES=native')
    $case = New-Case $Name $arguments
    $launch = $null
    $failure = $null
    try {
        $launch = Start-ProbeProcess $case.directory $arguments
        $window = Wait-Window $launch.Process {
            param($candidate)
            $candidate.visible -and -not $candidate.hasCaption -and
            $candidate.clientWidth -eq $candidate.monitorWidth -and
            $candidate.clientHeight -eq $candidate.monitorHeight
        } 'visible monitor-sized borderless startup'
        $case.snapshots += $window
        Assert-Condition ($window.className -eq 'SDL_app') "Window class is '$($window.className)', expected SDL_app"
        $case.assertions = @(
            'startup style has no WS_CAPTION',
            'startup client matches monitor dimensions',
            '-RES=native process remained running'
        )
    } catch {
        $failure = $_.Exception.Message
    } finally {
        Complete-Case $case $launch $failure
    }
}

[ordered]@{
    schemaVersion = 1
    runId = Split-Path -Leaf $runDir
    startedAtUtc = [DateTime]::UtcNow.ToString('o')
    executable = $exe
    executableSha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $exe).Hash
    gameDirectory = $game
    windowedClient = '960x540'
    restoreBoundsTolerancePixels = $RestoreBoundsTolerancePixels
    hiddenCapturesUsed = $false
    globalKeyboardInjectionUsed = $false
} | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $runDir 'environment.json') -Encoding utf8

$families = @([pscustomobject]@{ Name = 'direct-dirt1-jeep'; Prefix = @('DIRT1', 'JEEP') })
if ($IncludeMenu) { $families += [pscustomobject]@{ Name = 'menu'; Prefix = @() } }
if ($IncludeRuf1Jcb) { $families += [pscustomobject]@{ Name = 'direct-ruf1-jcb'; Prefix = @('RUF1', 'JCB') } }

$exitCode = 0
try {
    foreach ($family in $families) {
        $base = @($game) + @($family.Prefix) + @('-MUTE', '-NAME=VALIDATION')
        Test-RoundTrip "$($family.Name)-windowed-alt-enter-roundtrip" $base
        Test-BorderlessStartup "$($family.Name)-borderless-startup" $base
    }
} finally {
    # Per-case cleanup uses a try/finally. This outer pass is a final safety net,
    # and both paths use only process objects returned by Start-Process.
    foreach ($process in $createdProcesses) {
        try {
            $process.Refresh()
            if (-not $process.HasExited) { Stop-Process -Id $process.Id -Force -ErrorAction Stop }
        } catch {}
    }
    $failed = @($results | Where-Object { $_.status -ne 'PASS' }).Count
    $summary = [ordered]@{
        schemaVersion = 1
        finishedAtUtc = [DateTime]::UtcNow.ToString('o')
        status = if ($failed) { 'FAIL' } else { 'PASS' }
        cases = @($results)
        failedCaseCount = $failed
        createdProcessIds = @($createdPids)
        screenshotsProduced = $false
        proprietaryAssetsCopied = $false
    }
    $summary | ConvertTo-Json -Depth 14 | Set-Content -LiteralPath (Join-Path $runDir 'summary.json') -Encoding utf8
    Write-Output "OpenRedRacing window probe: $($summary.status)"
    Write-Output "Results: $runDir"
    foreach ($result in $results) {
        Write-Output "  $($result.status): $($result.name)$(
            if ($result.status -eq 'PASS') { '' } else { " - $($result.error)" })"
    }
    if ($failed) { $exitCode = 1 }
}

exit $exitCode
