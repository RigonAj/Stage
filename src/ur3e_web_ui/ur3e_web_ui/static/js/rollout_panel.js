import { api } from "./api.js";

const SETTINGS_FIELDS = [
  { key: "max_joint_velocity", id: "setting-max-joint-velocity", label: "Max velocity" },
  { key: "max_joint_acceleration", id: "setting-max-joint-acceleration", label: "Max acceleration" },
  { key: "approach_min_duration", id: "setting-approach-min-duration", label: "Approach min" },
  { key: "min_segment_duration", id: "setting-min-segment-duration", label: "Segment min" },
];

export class RolloutPanel {
  constructor({ viewer, onError }) {
    this.viewer = viewer;
    this.onError = onError;
    this.settings = null;
    this.selectedEpisode = null;
    this.currentPlan = null;
    this.executePlan = null;          // commanded plan being executed (blue ghost)
    this.executeReferencePlan = null; // other source overlaid during execution (orange ghost)
    this.executeReferenceSource = null;
    this.driverAlive = false;
    this.programRunning = null;
    this.lastGoalPhase = null;
    this.previewRequestId = 0;
    this.replaySource = "realized";

    document.getElementById("btn-settings-apply").addEventListener("click", () => this.applySettingsFromInputs());
    document.getElementById("setting-preview-approach").addEventListener("change", () => this.revalidateSelected());
    for (const radio of document.querySelectorAll('input[name="replay-source"]')) {
      radio.addEventListener("change", () => {
        if (!radio.checked) return;
        this.replaySource = radio.value;
        this.loadEpisodes();        // per-episode stats depend on the source
        this.revalidateSelected();  // refresh the open plan's stats too
      });
    }
    for (const button of document.querySelectorAll("[data-replay-preset]")) {
      button.addEventListener("click", () => this.applyPreset(button.dataset.replayPreset));
    }
    document.getElementById("btn-preview").addEventListener("click", () => this.preview());
    document.getElementById("btn-compare").addEventListener("click", () => this.compare());
    document.getElementById("btn-preview-stop").addEventListener("click", () => {
      this.previewRequestId += 1;
      this.viewer.stopPreview();
      this.togglePreviewButtons(false);
    });
    document.getElementById("btn-execute").addEventListener("click", () => this.openExecuteModal());
    document.getElementById("modal-cancel").addEventListener("click", () => this.closeModal());
    document.getElementById("modal-confirm").addEventListener("change", (event) => {
      document.getElementById("modal-go").disabled = !event.target.checked;
    });
    document.getElementById("modal-go").addEventListener("click", () => this.execute());
  }

  async load() {
    await this.loadSettings();
    await this.loadEpisodes();
  }

  async loadSettings() {
    try {
      const data = await api.get("/api/replay_settings");
      this.renderSettings(data);
    } catch (error) {
      this.onError(`replay settings: ${error.message}`);
    }
  }

  renderSettings(data) {
    this.settings = data;
    this.fillSettings(data.limits || {});
    for (const field of SETTINGS_FIELDS) {
      const input = document.getElementById(field.id);
      const bounds = (data.bounds || {})[field.key];
      if (!bounds) continue;
      input.min = bounds.min;
      input.max = bounds.max;
      input.step = bounds.step;
    }
    this.renderSettingsStatus();
  }

  fillSettings(values) {
    for (const field of SETTINGS_FIELDS) {
      const value = values[field.key];
      if (value === null || value === undefined) continue;
      document.getElementById(field.id).value = this.formatSetting(value);
    }
  }

  renderSettingsStatus(text = null) {
    const status = document.getElementById("settings-status");
    if (text) {
      status.textContent = text;
      return;
    }
    if (!this.settings) {
      status.textContent = "";
      return;
    }
    const limits = this.settings.limits;
    status.textContent =
      `active: ${limits.max_joint_velocity} rad/s, ${limits.max_joint_acceleration} rad/s^2, ` +
      `approach ${limits.approach_min_duration}s, segment ${limits.min_segment_duration}s`;
  }

  readSettingsInputs() {
    const values = {};
    for (const field of SETTINGS_FIELDS) {
      const value = Number.parseFloat(document.getElementById(field.id).value);
      if (!Number.isFinite(value)) throw new Error(`${field.label} must be a number`);
      values[field.key] = value;
    }
    return values;
  }

  async applySettingsFromInputs() {
    try {
      await this.applySettings(this.readSettingsInputs(), "settings applied");
    } catch (error) {
      this.onError(`settings: ${error.message}`);
    }
  }

  async applyPreset(name) {
    const preset = this.settings && this.settings.presets ? this.settings.presets[name] : null;
    if (!preset) return;
    try {
      await this.applySettings(preset, `${name} preset applied`);
    } catch (error) {
      this.onError(`preset: ${error.message}`);
    }
  }

  async applySettings(values, statusText) {
    const data = await api.post("/api/replay_settings", values);
    this.renderSettings(data);
    this.renderSettingsStatus(statusText);
    await this.loadEpisodes();
    await this.revalidateSelected();
  }

  async loadEpisodes() {
    try {
      const data = await api.get(`/api/rollout?source=${this.replaySource}`);
      const meta = data.metadata || {};
      document.getElementById("rollout-meta").textContent =
        `${data.path} — dt=${(meta.dt_s || 0).toFixed(4)}s, scale=${meta.action_scale}`;
      const tbody = document.querySelector("#episode-table tbody");
      tbody.innerHTML = "";
      for (const episode of data.episodes) {
        const row = document.createElement("tr");
        row.innerHTML = `
          <td>${episode.index}</td>
          <td>${episode.valid === false ? "invalid" : episode.success ? "&#10003;" : "&#10007;"}</td>
          <td class="num">${episode.steps}</td>
          <td class="num">${this.formatDuration(episode.retimed_total_s)}</td>
          <td><button data-episode="${episode.index}" ${episode.valid === false ? "disabled" : ""}>Validate</button></td>`;
        tbody.appendChild(row);
      }
      for (const button of tbody.querySelectorAll("button[data-episode]")) {
        button.addEventListener("click", () => this.validate(parseInt(button.dataset.episode, 10)));
      }
    } catch (error) {
      this.onError(`rollout: ${error.message}`);
    }
  }

  async validate(episodeIndex) {
    try {
      const includeApproach = document.getElementById("setting-preview-approach").checked && this.driverAlive;
      const approach = includeApproach ? "true" : "false";
      const plan = await api.get(`/api/rollout/${episodeIndex}/plan?approach=${approach}&source=${this.replaySource}`);
      this.selectedEpisode = episodeIndex;
      this.currentPlan = plan;
      this.renderStats(plan);
    } catch (error) {
      this.onError(`validate: ${error.message}`);
    }
  }

  async revalidateSelected() {
    if (this.selectedEpisode === null) return;
    await this.validate(this.selectedEpisode);
  }

  renderStats(plan) {
    document.getElementById("plan-stats").classList.remove("hidden");
    document.getElementById("plan-episode").textContent =
      `episode ${plan.episode_index} [${plan.source || this.replaySource}]` +
      (plan.approach_included ? " (with approach)" : " (no approach)");
    const set = (id, value) => { document.getElementById(id).textContent = value.toFixed(4); };
    set("st-raw-vel", plan.raw_stats.max_velocity_rad_s);
    set("st-ret-vel", plan.retimed_stats.max_velocity_rad_s);
    set("st-raw-acc", plan.raw_stats.max_acceleration_rad_s2);
    set("st-ret-acc", plan.retimed_stats.max_acceleration_rad_s2);
    set("st-raw-dur", plan.raw_stats.total_duration_s);
    set("st-ret-dur", plan.retimed_stats.total_duration_s);
    const limitsBox = document.getElementById("plan-limits");
    const limits = this.settings ? this.settings.limits : null;
    const limitText = limits
      ? ` Limits: ${limits.max_joint_velocity} rad/s, ${limits.max_joint_acceleration} rad/s^2.`
      : "";
    limitsBox.textContent = plan.within_limits
      ? `Retimed plan is within the configured safety limits.${limitText}`
      : `WARNING: retimed plan exceeds safety limits — execution will be refused.${limitText}`;
    limitsBox.style.color = plan.within_limits ? "" : "var(--danger)";
    document.getElementById("btn-execute").disabled = !plan.within_limits;
  }

  async preview() {
    if (!this.currentPlan) return;
    const requestId = this.previewRequestId + 1;
    this.previewRequestId = requestId;
    this.togglePreviewButtons(true);
    let referencePlan = null;
    if (this.currentPlan.approach_included && this.selectedEpisode !== null) {
      try {
        referencePlan = await api.get(
          `/api/rollout/${this.selectedEpisode}/plan?approach=false&source=${this.replaySource}`,
        );
      } catch (error) {
        this.onError(`recorded replay preview: ${error.message}`);
      }
    }
    if (requestId !== this.previewRequestId) return;
    const fill = document.getElementById("progress-fill");
    document.getElementById("exec-progress").classList.remove("hidden");
    document.getElementById("progress-label").textContent = referencePlan
      ? "preview: blue = real robot plan, amber = recorded replay"
      : "preview (ghost only — robot does not move)";
    this.viewer.playPreview(this.currentPlan, {
      referencePlan,
      onProgress: (fraction) => { fill.style.width = `${(fraction * 100).toFixed(1)}%`; },
      onDone: () => {
        this.togglePreviewButtons(false);
        document.getElementById("progress-label").textContent = "preview finished";
      },
    });
  }

  async compare() {
    if (this.selectedEpisode === null) return;
    const requestId = this.previewRequestId + 1;
    this.previewRequestId = requestId;
    this.togglePreviewButtons(true);
    let realizedPlan;
    let targetPlan;
    try {
      [realizedPlan, targetPlan] = await Promise.all([
        api.get(`/api/rollout/${this.selectedEpisode}/plan?approach=false&source=realized`),
        api.get(`/api/rollout/${this.selectedEpisode}/plan?approach=false&source=target`),
      ]);
    } catch (error) {
      this.onError(`compare: ${error.message}`);
      this.togglePreviewButtons(false);
      return;
    }
    if (requestId !== this.previewRequestId) return;
    // Both sources have one sample per control step, so they are naturally
    // frame-aligned. Replay them over a fixed, watchable duration (the raw
    // episode is only a fraction of a second) so the difference is visible:
    // the policy command (orange) whips far while the realized motion (blue)
    // stays small and smooth.
    const COMPARE_VISIBLE_S = 4.0;
    const frames = Math.max(realizedPlan.positions.length, targetPlan.positions.length);
    const step = frames > 1 ? COMPARE_VISIBLE_S / (frames - 1) : COMPARE_VISIBLE_S;
    const stretch = (plan) => ({
      joint_names: plan.joint_names,
      positions: plan.positions,
      time_from_start_s: plan.positions.map((_, i) => i * step),
    });
    const fill = document.getElementById("progress-fill");
    document.getElementById("exec-progress").classList.remove("hidden");
    document.getElementById("progress-label").textContent =
      "compare (slowed): blue = realized (sim motion), orange = target (policy command)";
    this.viewer.playPreview(stretch(realizedPlan), {
      referencePlan: stretch(targetPlan),
      onProgress: (fraction) => { fill.style.width = `${(fraction * 100).toFixed(1)}%`; },
      onDone: () => {
        this.togglePreviewButtons(false);
        document.getElementById("progress-label").textContent = "compare finished";
      },
    });
  }

  togglePreviewButtons(playing) {
    document.getElementById("btn-preview").classList.toggle("hidden", playing);
    document.getElementById("btn-compare").classList.toggle("hidden", playing);
    document.getElementById("btn-preview-stop").classList.toggle("hidden", !playing);
  }

  async openExecuteModal() {
    if (this.selectedEpisode === null) return;
    try {
      // Recompute with a fresh approach segment so the modal shows what will actually run.
      const plan = await api.get(
        `/api/rollout/${this.selectedEpisode}/plan?approach=true&source=${this.replaySource}`,
      );
      this.currentPlan = plan;
      const body = document.getElementById("modal-body");
      const warning = this.programRunning === false
        ? `<p style="color: var(--danger); font-weight: 700;">External Control program is NOT running on the
           pendant — the goal will be accepted but the robot will not move.</p>`
        : "";
      const limits = this.settings ? this.settings.limits : null;
      const limitsText = limits
        ? `Limits: ${limits.max_joint_velocity} rad/s, ${limits.max_joint_acceleration} rad/s&sup2;,
           approach ${limits.approach_min_duration}s, segment ${limits.min_segment_duration}s.`
        : "";
      body.innerHTML = `${warning}
        <p>Episode <b>${plan.episode_index}</b> — source <b>${plan.source || this.replaySource}</b>:
        ${plan.positions.length} points,
        total <b>${plan.retimed_stats.total_duration_s.toFixed(1)} s</b>
        (${plan.approach_included ? "incl. live approach" : "no approach"}), max velocity
        <b>${plan.retimed_stats.max_velocity_rad_s.toFixed(3)} rad/s</b>, max acceleration
        <b>${plan.retimed_stats.max_acceleration_rad_s2.toFixed(3)} rad/s&sup2;</b>.</p>
        <p class="hint">${limitsText}</p>`;
      document.getElementById("modal-confirm").checked = false;
      document.getElementById("modal-go").disabled = true;
      document.getElementById("modal").classList.remove("hidden");
    } catch (error) {
      this.onError(`plan: ${error.message}`);
    }
  }

  closeModal() {
    document.getElementById("modal").classList.add("hidden");
  }

  async execute() {
    this.closeModal();
    try {
      // Snapshot the plans overlaid as ghosts while the real robot executes.
      // Blue = the commanded trajectory being executed; orange = the other
      // source (the raw policy command), fetched with the same live approach
      // and from the current pose *before* the robot starts moving so both
      // ghosts share a start pose and stay aligned by progress fraction.
      this.executePlan = this.currentPlan;
      this.executeReferenceSource = this.replaySource === "target" ? "realized" : "target";
      this.executeReferencePlan = null;
      try {
        this.executeReferencePlan = await api.get(
          `/api/rollout/${this.selectedEpisode}/plan?approach=true&source=${this.executeReferenceSource}`,
        );
      } catch (error) {
        this.executeReferencePlan = null; // best-effort overlay; execution still proceeds
      }
      await api.post(`/api/rollout/${this.selectedEpisode}/execute`, { confirm: true, source: this.replaySource });
      document.getElementById("exec-progress").classList.remove("hidden");
    } catch (error) {
      this.onError(`execute: ${error.message}`);
    }
  }

  update(state) {
    this.driverAlive = !!(state.driver && state.driver.joint_states_alive);
    this.programRunning = state.driver ? state.driver.program_running : null;

    const goal = state.goal || { phase: "idle" };
    if (goal.kind === "rollout" && goal.phase !== "idle") {
      const progressBox = document.getElementById("exec-progress");
      const fill = document.getElementById("progress-fill");
      const label = document.getElementById("progress-label");
      if (goal.phase === "active" && goal.total_s > 0) {
        const fraction = Math.min(1, goal.elapsed_s / goal.total_s);
        progressBox.classList.remove("hidden");
        fill.style.width = `${(fraction * 100).toFixed(1)}%`;
        const legend = this.executePlan
          ? (this.executeReferencePlan
            ? ` — ghosts: blue = ${this.replaySource} (executed), orange = ${this.executeReferenceSource}`
            : ` — blue ghost = ${this.replaySource} (executed)`)
          : "";
        label.textContent =
          `executing episode ${goal.episode}: ${goal.elapsed_s.toFixed(1)}/${goal.total_s.toFixed(1)} s${legend}`;
        this.lastGoalPhase = goal.phase;
        // Overlay the commanded trajectory (and reference source) on the live
        // robot, advanced by execution progress so it tracks the real motion.
        if (this.executePlan) {
          this.viewer.showExecutionGhosts(this.executePlan, this.executeReferencePlan, fraction);
        }
      } else if (
        ["succeeded", "aborted", "canceled", "rejected"].includes(goal.phase)
        && this.lastGoalPhase !== goal.phase
      ) {
        // Render terminal states once so they don't clobber preview text later.
        progressBox.classList.remove("hidden");
        if (goal.phase === "succeeded") fill.style.width = "100%";
        label.textContent = `episode ${goal.episode}: ${goal.phase}` +
          (goal.error_string && goal.error_code !== 0 ? ` — ${goal.error_string}` : "");
        this.lastGoalPhase = goal.phase;
        this.viewer.hideExecutionGhosts();
        this.executePlan = null;
        this.executeReferencePlan = null;
      }
    }
  }

  formatSetting(value) {
    return Number.parseFloat(value).toFixed(3).replace(/\.?0+$/, "");
  }

  formatDuration(value) {
    return Number.isFinite(value) ? Number.parseFloat(value).toFixed(3) : "–";
  }
}
