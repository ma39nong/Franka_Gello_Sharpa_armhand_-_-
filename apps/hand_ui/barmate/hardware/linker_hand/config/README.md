# Linker Hand preset configuration design

This directory is the project-owned configuration home for Linker Hand UI
presets and tasks. It intentionally does not read from, reuse, or mutate
`barmate/vendor/linkerhand_sdk/config`.

Preset is the only source of hand poses for the UI. Built-in presets from code
and vendor presets are not loaded by the future UI preset system. If a default
pose should appear in the UI, it should be represented in one of the files in
this directory.

## Storage location

Use one configuration file per UI hand side:

```text
barmate/hardware/linker_hand/config/left_hand.yaml
barmate/hardware/linker_hand/config/right_hand.yaml
```

This keeps each file short and makes it natural to copy, review, or replace one
hand's preset library without touching the other hand.

The implementation should create `barmate/hardware/linker_hand/config/` and the
side-specific file on first save if they do not exist.

## Domain model

Preset is the fundamental persisted entity.

A preset is one named hand pose for one hand side and one hand model. It owns the
joint values that can be sent to the hand.

Task is only a user-facing grouping/view over presets.

Action sequence is a second persisted control entity. It is an ordered list of
`timestamp -> preset` steps for one hand side and one hand model. A sequence
does not embed hand values; every step references an existing preset ID.

A task does not own presets or action sequences. Deleting a task must not delete
the entities it references. A single preset or action sequence may appear in
multiple tasks.

## File-level schema

Recommended schema version: `1`.

Each file represents exactly one UI side. `side` must match the file name:

- `left_hand.yaml` uses `side: LEFT_HAND`
- `right_hand.yaml` uses `side: RIGHT_HAND`

Example `left_hand.yaml`:

```yaml
schema_version: 1
side: LEFT_HAND

models:
  G20:
    presets:
      - id: "grasp-lemon-ready"
        name: "抓柠檬-准备"
        values: [128, 10, 20, 30, 40, 128, 0, 0, 0, 0, 128, 0, 0, 0, 0, 128, 0, 0, 0, 0]

      - id: "grasp-lemon-close"
        name: "抓柠檬-闭合"
        values: [128, 80, 90, 100, 110, 128, 90, 90, 90, 90, 128, 90, 90, 90, 90, 128, 80, 80, 80, 80]

    tasks:
      - id: "lemon-task"
        name: "柠檬任务"
        preset_ids:
          - "grasp-lemon-ready"
          - "grasp-lemon-close"
        action_sequence_ids:
          - "grasp-lemon-sequence"

      - id: "demo-task"
        name: "演示任务"
        preset_ids:
          - "grasp-lemon-ready"

    action_sequences:
      - id: "grasp-lemon-sequence"
        name: "抓柠檬序列"
        steps:
          - timestamp: 0.0
            preset_id: "grasp-lemon-ready"
          - timestamp: 1.2
            preset_id: "grasp-lemon-close"
```

Example `right_hand.yaml`:

```yaml
schema_version: 1
side: RIGHT_HAND

models:
  L20:
    presets:
      - id: "hold-cup"
        name: "握杯"
        values: [128, 60, 70, 80, 90, 128, 60, 70, 80, 90, 128, 60, 70, 80, 90, 128, 60, 70, 80, 90]

    tasks:
      - id: "pour-task"
        name: "倒酒任务"
        preset_ids:
          - "hold-cup"
```

## Key rules

1. Each file stores exactly one hand side.
2. `models` is keyed by hand model, for example `G20`, `L20`, `L10`.
3. Preset IDs are unique within one `side + model` scope.
4. Task IDs are unique within one `side + model` scope.
5. A task stores only `preset_ids` and `action_sequence_ids`; it must not embed
   preset values or sequence steps.
6. A preset may be referenced by zero, one, or many tasks.
7. Deleting a task removes only the task entry.
8. Deleting a preset removes the preset entry and should also remove its ID from
   all tasks in the same `side + model` scope.
9. Preset values should be normalized to the model joint count on load and save:
   - clamp every value to `0..255`
   - truncate extra values
   - pad missing values with `0`

## UI behavior mapping

Current UI concepts should map like this:

- `ADD TASK`: create a task entry under the current `side + model` scope in that
  side's file.
- `SAVE PRESET`: create a new preset from the current hand values and add that
  preset ID to the active task.
- `ADD ACTION SEQUENCE`: create a named timestamp -> preset sequence and add
  that sequence ID to the active task.
- Applying a preset: look up the selected preset ID, then send the preset values.
- Applying an action sequence: play its steps by timestamp, looking up and
  sending each referenced preset.
- Removing a task: remove only the task entry.
- Removing a preset: remove the preset entity and clean all task references to it.

If the UI later supports assigning an existing preset to another task, it should
append the existing preset ID to that task's `preset_ids` rather than duplicating
the preset.

## Load and save behavior

The UI should load presets only from this directory:

```text
barmate/hardware/linker_hand/config/left_hand.yaml
barmate/hardware/linker_hand/config/right_hand.yaml
```

There is no merge with code-defined presets or vendor presets. The config files
are the complete preset/task source for the UI.

Writes should update only the side-specific file for the hand being edited. For
example, saving a left-hand preset should update `left_hand.yaml` and leave
`right_hand.yaml` untouched.

## Implementation notes for current code

The current in-memory shape is task-owned:

```python
preset_groups: OrderedDict[str, list[HandPreset]]
```

For this design, the future in-memory shape should separate entities from views,
for example:

```python
presets: OrderedDict[str, HandPreset]
tasks: OrderedDict[str, list[str]]  # task_id -> preset_id list
active_task: str
```

The existing UI can still render a tab per task. At render time, each task's
`preset_ids` should be resolved to preset objects.

## Non-goals

- Do not read from or write to vendor config.
- Do not load code-defined presets.
- Do not make task deletion cascade to presets.
- Do not require task names to be globally unique across sides or models.
- Do not require preset names to be globally unique; stable IDs are the identity.
