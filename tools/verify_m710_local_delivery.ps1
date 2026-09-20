param([Parameter(Mandatory=$true)][string]$DeliveryDirectory)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path -LiteralPath $DeliveryDirectory).Path
$manifest = Get-Content -Encoding UTF8 -LiteralPath (Join-Path $root 'delivery_manifest.json') -Raw | ConvertFrom-Json
$verified = @()
Add-Type -AssemblyName System.Drawing
foreach ($entry in $manifest.files) {
    $path = [IO.Path]::GetFullPath((Join-Path $root $entry.path))
    if (-not $path.StartsWith($root + [IO.Path]::DirectorySeparatorChar)) { throw "Path escapes delivery: $path" }
    $file = Get-Item -LiteralPath $path
    if ($file.Length -ne $entry.bytes -or $file.Length -eq 0) { throw "Size mismatch: $path" }
    $hash = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($hash -ne $entry.sha256) { throw "SHA-256 mismatch: $path" }
    if ($file.Extension -eq '.png') {
        $bitmap = [Drawing.Image]::FromFile($path)
        try { if ($bitmap.Width -le 0 -or $bitmap.Height -le 0) { throw "Invalid image: $path" } }
        finally { $bitmap.Dispose() }
    }
    if ($file.Extension -eq '.mp4' -and (-not $entry.video_audit.valid -or $entry.video_audit.decoded_frames -le 0)) {
        throw "No full server decode proof for hash-identical video: $path"
    }
    $verified += [ordered]@{path=$entry.path; bytes=$file.Length; sha256=$hash}
}
$index = Get-Content -Encoding UTF8 -LiteralPath (Join-Path $root 'index.html') -Raw
foreach ($match in [regex]::Matches($index, '(?:src|href)="([^"]+)"')) {
    $relative = $match.Groups[1].Value
    if ($relative -match '^([a-z]+:|/|\\)') { throw "Nonlocal index reference: $relative" }
    if (-not (Test-Path -LiteralPath (Join-Path $root $relative))) { throw "Missing index reference: $relative" }
}
$result = [ordered]@{status='PASS'; local_directory=$root; world_session_id=$manifest.world_session_id;
    files_verified=$verified.Count; videos=$manifest.videos; original_videos=$manifest.original_videos;
    png_files=$manifest.png_files; unique_png_images=$manifest.unique_png_images;
    extracted_event_frames=$manifest.extracted_event_frames;
    video_verification='Full server decode plus local byte count and SHA-256 equality'; files=$verified}
$result | ConvertTo-Json -Depth 8 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $root 'local_verification.json')
[pscustomobject]$result | Select-Object status,local_directory,world_session_id,files_verified,videos,png_files | Format-List
