# AGENTS.md

This repository runs in minimal mode: skills-first execution with shared project templates.

Operating model:
- You are the product owner.
- The AI system acts as your tech cofounder/expert.
- Each skill acts like a specialized junior worker with strict role boundaries.

## Core Artifacts

- `AGENTS.md`: orchestration entrypoint and delegation rules.
- `project_templates/PROJECT_CONTEXT.md`: required project scope and constraints.
- `skills/`: reusable execution skills (role skills + process skills).

## Start Here Every Task

1. Load `AGENTS.md`.
2. Load `project_templates/PROJECT_CONTEXT.md` (or the project-specific `PROJECT_CONTEXT.md`).
3. Pick one lead role skill for the task.
4. Attach 1-2 supporting process skills only when needed.
5. Require handoff output if the task must be delegated.

Prompt pattern:
```text
Follow AGENTS.md, skills/<ROLE_SKILL>/SKILL.md, and PROJECT_CONTEXT.md.
Goal: ...
Deliverables: ...
Constraints: ...
```

## Delegation Contracts

Every skill must produce:
- Scope handled.
- Decisions made and why.
- Open risks/blockers.
- Exact handoff target (next skill + expected deliverable).

## Role Skills

- `skills/architect-owner/`
- `skills/system-designer/`
- `skills/backend-engineer/`
- `skills/frontend-engineer/`
- `skills/data-engineer-analyst/`
- `skills/qa-test-engineer/`
- `skills/debug-reliability/`
- `skills/docs-knowledge/`
- `skills/reviewer-guardrails/`
- `skills/research-decisions/`
- `skills/learning-engineer/`

## Process/Utility Skills

Attach only when useful:
- `skills/repo_analysis.md`
- `skills/code_generation.md`
- `skills/debugging.md`
- `skills/documentation.md`
- `skills/experiment_design.md`
- `skills/handoff-packet/`
- `skills/handoff-ingestion/`
- `skills/git-commit-push/`

## Recommended Flow

Default end-to-end flow:
1. Architect Owner
2. System Designer (if architecture/interface clarity is needed)
3. Backend and/or Frontend Engineer
4. QA Test Engineer
5. Reviewer Guardrails
6. Docs Knowledge
7. Learning Engineer

Conditional roles:
- Use Debug Reliability when broken/unstable.
- Use Data Engineer Analyst when metrics/data pipelines are required.
- Use Research Decisions when external evidence or comparison is needed.

## Learning System

Learning is mandatory for meaningful fixes, incidents, or major decision reversals.

Use:
- `skills/learning-engineer/`
- `project_templates/LEARNING_LOG.md`

Record:
- Cause
- Action
- Result
- Reusable prevention/check

## Canonical Governance

Use this repo as canonical for reusable system quality, not project-specific delivery state.

Global artifacts (canonical):
- `AGENTS.md`
- `skills/**`
- `project_templates/**` templates only
- `usage/**`

Project-local artifacts (do not promote by default):
- Filled `PROJECT_CONTEXT.md`
- `tasks.md`, `worklog.md`, `decisions.md`, `LEARNING_LOG.md` with project facts
- Handoff packets and project research notes
- Any client/domain-specific constraints that do not generalize

## Promotion Gate

Promote project learnings into canonical only when all are true:
1. Reusable across at least 2 projects.
2. Improves reliability, clarity, or speed without adding project coupling.
3. Can be written as a general rule/checklist/example.
4. Does not include sensitive or project-identifying data.

If uncertain, keep it local and add a follow-up candidate in `project_templates/LEARNING_LOG.md`.

## Safety Rails

Before editing canonical skill files:
1. State what reusable behavior is being improved.
2. State why the change is not project-specific.
3. Record expected impact on quality/speed/risk.
4. Keep edits minimal and scoped.

Before promoting any local content:
1. Remove project names, credentials, customer details, and internal identifiers.
2. Convert concrete project facts into abstract patterns.
3. Add a short rationale for reusability.
4. Validate no active project assumptions remain.
