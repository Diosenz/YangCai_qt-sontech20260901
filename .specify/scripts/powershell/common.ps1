Set-StrictMode -Version Latest

function Get-SpecifyRepositoryRoot {
    [CmdletBinding()]
    param()

    # This file lives at <repository>/.specify/scripts/powershell.
    $repositoryRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
    $specifyDirectory = Join-Path $repositoryRoot '.specify'
    if (-not (Test-Path -LiteralPath $specifyDirectory -PathType Container)) {
        throw "Cannot locate the repository .specify directory from '$PSScriptRoot'."
    }

    return [System.IO.Path]::GetFullPath($repositoryRoot)
}

function Get-SpecifyBranchName {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)]
        [string]$RepositoryRoot
    )

    try {
        $branchName = & git -C $RepositoryRoot branch --show-current 2>$null
        if (-not [string]::IsNullOrWhiteSpace($branchName)) {
            return $branchName.Trim()
        }
    }
    catch {
        # A detached worktree or a source archive is still usable when feature.json is present.
    }

    return 'detached-or-unavailable'
}

function ConvertTo-CanonicalAbsolutePath {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        [string]$BasePath
    )

    if ([System.IO.Path]::IsPathRooted($Path)) {
        return [System.IO.Path]::GetFullPath($Path)
    }

    return [System.IO.Path]::GetFullPath((Join-Path $BasePath $Path))
}

function Get-SpecifyFeatureContext {
    [CmdletBinding()]
    param()

    $repositoryRoot = Get-SpecifyRepositoryRoot
    $featureConfig = Join-Path $repositoryRoot '.specify\feature.json'
    if (-not (Test-Path -LiteralPath $featureConfig -PathType Leaf)) {
        throw "Missing '$featureConfig'. Run /speckit-specify to select or create a feature first."
    }

    try {
        $featureSelection = Get-Content -LiteralPath $featureConfig -Raw -Encoding UTF8 | ConvertFrom-Json -ErrorAction Stop
    }
    catch {
        throw "Cannot parse '$featureConfig': $($_.Exception.Message)"
    }

    $configuredFeatureDirectory = [string]$featureSelection.feature_directory
    if ([string]::IsNullOrWhiteSpace($configuredFeatureDirectory)) {
        throw "'$featureConfig' must contain a non-empty feature_directory value."
    }

    $featureDirectory = ConvertTo-CanonicalAbsolutePath -Path $configuredFeatureDirectory -BasePath $repositoryRoot
    if (-not (Test-Path -LiteralPath $featureDirectory -PathType Container)) {
        throw "The selected feature directory does not exist: '$featureDirectory'."
    }

    $availableDocs = Get-SpecifyAvailableDocuments -FeatureDirectory $featureDirectory
    return [PSCustomObject]@{
        RepositoryRoot = $repositoryRoot
        Branch = Get-SpecifyBranchName -RepositoryRoot $repositoryRoot
        FeatureDirectory = $featureDirectory
        FeatureSpec = Join-Path $featureDirectory 'spec.md'
        ImplPlan = Join-Path $featureDirectory 'plan.md'
        Tasks = Join-Path $featureDirectory 'tasks.md'
        AvailableDocs = $availableDocs
    }
}

function Get-SpecifyAvailableDocuments {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)]
        [string]$FeatureDirectory
    )

    $documents = [System.Collections.Generic.List[string]]::new()
    Get-ChildItem -LiteralPath $FeatureDirectory -File -Recurse | ForEach-Object {
        $relativePath = $_.FullName.Substring($FeatureDirectory.Length).TrimStart('\', '/')
        $documents.Add($relativePath.Replace('\', '/'))
    }

    Get-ChildItem -LiteralPath $FeatureDirectory -Directory | ForEach-Object {
        $documents.Add(($_.Name + '/'))
    }

    return @($documents | Sort-Object -Unique)
}

function Assert-SpecifyPrerequisites {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)]
        [psobject]$Context,

        [switch]$RequireTasks
    )

    $missingPaths = [System.Collections.Generic.List[string]]::new()
    if (-not (Test-Path -LiteralPath $Context.FeatureSpec -PathType Leaf)) {
        $missingPaths.Add('spec.md (run /speckit-specify)')
    }

    if ($RequireTasks) {
        if (-not (Test-Path -LiteralPath $Context.ImplPlan -PathType Leaf)) {
            $missingPaths.Add('plan.md (run /speckit-plan)')
        }
        if (-not (Test-Path -LiteralPath $Context.Tasks -PathType Leaf)) {
            $missingPaths.Add('tasks.md (run /speckit-tasks)')
        }
    }

    if ($missingPaths.Count -gt 0) {
        throw "Speckit prerequisites are missing: $($missingPaths -join '; ')."
    }
}

function Write-SpecifyPayload {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)]
        [System.Collections.IDictionary]$Payload,

        [switch]$Json
    )

    if ($Json) {
        $Payload | ConvertTo-Json -Depth 5 -Compress
        return
    }

    $Payload.GetEnumerator() |
        Sort-Object Name |
        ForEach-Object { '{0}: {1}' -f $_.Key, ($_.Value -join ', ') }
}
