[CmdletBinding()]
param(
    [switch]$Json,
    [switch]$RequireTasks,
    [switch]$IncludeTasks,
    [switch]$PathsOnly
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')

try {
    $context = Get-SpecifyFeatureContext
    if (-not $PathsOnly) {
        Assert-SpecifyPrerequisites -Context $context -RequireTasks:$RequireTasks
    }

    $payload = [ordered]@{
        REPO_ROOT = $context.RepositoryRoot
        BRANCH = $context.Branch
        FEATURE_DIR = $context.FeatureDirectory
        FEATURE_SPEC = $context.FeatureSpec
        IMPL_PLAN = $context.ImplPlan
        TASKS = $context.Tasks
    }

    if (-not $PathsOnly) {
        $payload.AVAILABLE_DOCS = $context.AvailableDocs
    }

    Write-SpecifyPayload -Payload $payload -Json:$Json
}
catch {
    Write-Error $_.Exception.Message
    exit 1
}
