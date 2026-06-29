# Testing And Commands

> Sources: repository README, 2026-06-29; live-catch README, 2026-06-29; implementation status, 2026-06-29; web UI docs, 2026-06-29
> Raw: [README](../../README.md); [Live-catch README](../../src/ur3e_live_catch/README.md); [Implementation status](../../docs/Robot_Control/ur3e_live_catch_implementation_status.md); [Web UI docs](../../docs/Robot_Control/ur3e_web_ui.md)

## Environment

```bash
source env.sh
```

## Build

```bash
build
colcon build --packages-select ur3e_catch_msgs ur3e_live_catch
colcon build --packages-select ball_tracking_cpp
```

## Run

```bash
run
ur3e_stack
ur3e_catch_stack
```

## Tests

```bash
cd src/ur3e_live_catch && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_rollout_replay && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_web_ui && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
cd src/ur3e_sysid && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

## Wiki Maintenance

```bash
python3 scripts/lint_llm_wiki.py
python3 scripts/update_agent_wiki.py
```

## See Also

- [Wiki Maintenance](wiki-maintenance.md)
- [Source Document Map](source-document-map.md)
- [Current Status And Blockers](../live-catch/current-status-and-blockers.md)
