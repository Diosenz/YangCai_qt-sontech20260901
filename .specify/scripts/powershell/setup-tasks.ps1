[CmdletBinding()]
param(
    [switch]$Json
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')

try {
    $context = Get-SpecifyFeatureContext
    if (-not (Test-Path -LiteralPath $context.FeatureSpec -PathType Leaf)) {
        throw "Missing '$($context.FeatureSpec)'. Run /speckit-specify before /speckit-tasks."
    }
    if (-not (Test-Path -LiteralPath $context.ImplPlan -PathType Leaf)) {
        throw "Missing '$($context.ImplPlan)'. Run /speckit-plan before /speckit-tasks."
    }

    $tasksTemplate = Join-Path $context.RepositoryRoot '.specify\templates\tasks-template.md'
    if (-not (Test-Path -LiteralPath $tasksTemplate -PathType Leaf)) {
        throw "Missing tasks template '$tasksTemplate'."
    }

    $payload = [ordered]@{
        REPO_ROOT = $context.RepositoryRoot
        FEATURE_DIR = $context.FeatureDirectory
        FEATURE_SPEC = $context.FeatureSpec
        IMPL_PLAN = $context.ImplPlan
        TASKS = $context.Tasks
        TASKS_TEMPLATE = $tasksTemplate
        AVAILABLE_DOCS = $context.AvailableDocs
    }

    Write-SpecifyPayload -Payload $payload -Json:$Json
}
catch {
    Write-Error $_.Exception.Message
    exit 1
}
