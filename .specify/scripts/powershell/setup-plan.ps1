[CmdletBinding()]
param(
    [switch]$Json
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')

try {
    $context = Get-SpecifyFeatureContext
    if (-not (Test-Path -LiteralPath $context.FeatureSpec -PathType Leaf)) {
        throw "Missing '$($context.FeatureSpec)'. Run /speckit-specify before /speckit-plan."
    }

    if (-not (Test-Path -LiteralPath $context.ImplPlan -PathType Leaf)) {
        $planTemplate = Join-Path $context.RepositoryRoot '.specify\templates\plan-template.md'
        if (-not (Test-Path -LiteralPath $planTemplate -PathType Leaf)) {
            throw "Missing plan template '$planTemplate'."
        }

        Copy-Item -LiteralPath $planTemplate -Destination $context.ImplPlan -ErrorAction Stop
    }

    $payload = [ordered]@{
        REPO_ROOT = $context.RepositoryRoot
        BRANCH = $context.Branch
        SPECS_DIR = Join-Path $context.RepositoryRoot 'specs'
        FEATURE_DIR = $context.FeatureDirectory
        FEATURE_SPEC = $context.FeatureSpec
        IMPL_PLAN = $context.ImplPlan
    }

    Write-SpecifyPayload -Payload $payload -Json:$Json
}
catch {
    Write-Error $_.Exception.Message
    exit 1
}
