import { api } from "./api.js";

function fmtVec(arr, digits = 3) {
  if (!arr || !arr.length) return "–";
  return arr.map((v) => Number(v).toFixed(digits)).join(", ");
}

function fmtMs(seconds, digits = 0) {
  return seconds === null || seconds === undefined ? "–" : `${(seconds * 1000).toFixed(digits)} ms`;
}

const DEFAULT_BALL_CONFIG = {
  p0: [-1.0, 1.5, 0.4],
  v0: [1.5, -1.0, 2.5],
  flight_s: 2.0,
};

const ISAAC_RANDOM_BALL = {
  p0Ranges: [[-0.6, -0.2], [1.2, 2.1], [0.5, 1.2]],
  v0Ranges: [[-0.7, 0.6], [-5.0, -3.5], [-0.1, 1.5]],
  positionNoiseStd: 0.01,
};

const BALL_FIELD_IDS = {
  p0: ["catch-p0-x", "catch-p0-y", "catch-p0-z"],
  v0: ["catch-v0-x", "catch-v0-y", "catch-v0-z"],
};
const FLIGHT_FIELD_ID = "catch-flight-s";  // restart_after_s (flight duration, s)

function uniform([lo, hi]) {
  return lo + Math.random() * (hi - lo);
}

function gaussian(std) {
  if (std <= 0) return 0;
  const u1 = Math.max(Number.MIN_VALUE, Math.random());
  const u2 = Math.random();
  return std * Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
}

// Test tab: throw a virtual ball at the live-catch policy and (optionally) let the
// policy command the real robot. The green "policy ghost" in the 3D view is driven
// by CatchTelemetry.joint_target inside viewer.setCatch() — this panel only triggers
// the ROS services and renders the telemetry / button state.
export class CatchPanel {
  constructor({ viewer, onError }) {
    this.viewer = viewer;
    this.onError = onError;
    this.commandReady = false;
    this.configReady = false;
    this.commandEnabled = false;
    this.configLoaded = false;
    this.mutingInputs = false;
    this.ballConfig = {
      p0: [...DEFAULT_BALL_CONFIG.p0],
      v0: [...DEFAULT_BALL_CONFIG.v0],
      flight_s: DEFAULT_BALL_CONFIG.flight_s,
    };

    this.viewer.enableBallLaunchFrame((config) => this.onLaunchFrameDragged(config));
    this.writeBallConfig(this.ballConfig);
    this.viewer.setBallLaunchConfig(this.ballConfig);

    for (const id of [...BALL_FIELD_IDS.p0, ...BALL_FIELD_IDS.v0, FLIGHT_FIELD_ID]) {
      document.getElementById(id).addEventListener("input", () => this.onBallInputChanged());
    }
    document.getElementById("btn-catch-apply-ball").addEventListener("click", () => this.applyBallConfig());
    document.getElementById("btn-catch-reset-ball").addEventListener("click", () => this.resetBallConfig());
    document.getElementById("btn-catch-isaac-random").addEventListener("click", () => this.throwIsaacRandomBall());
    document.getElementById("btn-catch-throw").addEventListener("click", () => this.throwBall());
    document.getElementById("btn-catch-command-on").addEventListener("click", () => this.setCommand(true));
    document.getElementById("btn-catch-command-off").addEventListener("click", () => this.setCommand(false));
    document.getElementById("catch-confirm").addEventListener("change", () => this.refreshButtons());
    document.getElementById("catch-show-launch-frame").addEventListener("change", (event) => {
      this.viewer.setBallLaunchFrameVisible(event.target.checked);
    });
    document.getElementById("catch-show-ghost").addEventListener("change", (event) => {
      this.viewer.setPolicyGhostEnabled(event.target.checked);
    });
    for (const button of document.querySelectorAll("[data-launch-mode]")) {
      button.addEventListener("click", () => this.setLaunchMode(button.dataset.launchMode));
    }
    this.setLaunchMode("move");
  }

  // Pick which gizmo the mouse drives: move p0 (translate) | aim v0 (rotate) | speed |v0| (scale).
  setLaunchMode(mode) {
    this.viewer.setBallLaunchMode(mode);
    for (const button of document.querySelectorAll("[data-launch-mode]")) {
      button.classList.toggle("active", button.dataset.launchMode === mode);
    }
  }

  async loadBallConfig() {
    try {
      const result = await api.get("/api/catch/ball_config");
      this.setBallConfig(result, "launch frame loaded from test_ball_node");
      this.configLoaded = true;
    } catch (error) {
      this.setText("catch-status", `launch frame local (${error.message})`);
    }
  }

  async applyBallConfig() {
    try {
      const config = this.readBallConfig();
      const result = await api.post("/api/catch/ball_config", config);
      this.setBallConfig(result, result.message || "launch frame applied");
      this.configLoaded = true;
    } catch (error) {
      this.onError(`ball config: ${error.message}`);
    }
  }

  resetBallConfig() {
    this.setBallConfig(DEFAULT_BALL_CONFIG, "launch frame reset");
  }

  sampleIsaacRandomBallConfig() {
    return {
      p0: ISAAC_RANDOM_BALL.p0Ranges.map(
        (range) => uniform(range) + gaussian(ISAAC_RANDOM_BALL.positionNoiseStd),
      ),
      v0: ISAAC_RANDOM_BALL.v0Ranges.map((range) => uniform(range)),
      flight_s: this.ballConfig.flight_s,
    };
  }

  async throwIsaacRandomBall() {
    const config = this.sampleIsaacRandomBallConfig();
    this.setBallConfig(config, "Isaac random sampled");
    try {
      const result = await api.post("/api/catch/throw", config);
      this.setText("catch-status", result.message || "Isaac random ball thrown");
    } catch (error) {
      this.onError(`Isaac random throw: ${error.message}`);
    }
  }

  async throwBall() {
    try {
      const result = await api.post("/api/catch/throw", this.readBallConfig());
      this.setText("catch-status", result.message || "ball thrown");
    } catch (error) {
      this.onError(`throw: ${error.message}`);
    }
  }

  async setCommand(enable) {
    if (enable && !document.getElementById("catch-confirm").checked) return;
    try {
      const result = await api.post("/api/catch/command", { enable, confirm: enable });
      this.setText("catch-command-status", result.message || (enable ? "command on" : "command off"));
      if (!enable) document.getElementById("catch-confirm").checked = false;
    } catch (error) {
      this.onError(`command: ${error.message}`);
    }
  }

  update(state) {
    // goal_event frames carry only {goal}; skip them so the buttons don't flicker.
    if (!state.catch_status) return;
    const status = state.catch_status;
    this.commandReady = !!status.command_ready;
    this.configReady = !!status.config_ready;
    this.commandEnabled = !!status.command_enabled;

    if (this.configReady && !this.configLoaded) this.loadBallConfig();

    document.getElementById("btn-catch-throw").disabled = !status.throw_ready || !this.configReady;
    document.getElementById("btn-catch-apply-ball").disabled = !this.configReady;
    document.getElementById("btn-catch-isaac-random").disabled = !status.throw_ready || !this.configReady;
    this.refreshButtons();

    const cmdStatus = document.getElementById("catch-command-status");
    cmdStatus.textContent = this.commandEnabled
      ? "command: ON — the policy is driving the real robot"
      : `command: off${this.commandReady ? "" : " (live_catch node not found)"}`;
    cmdStatus.className = "goal-status " + (this.commandEnabled ? "active" : "");

    const badge = document.getElementById("badge-catch");
    if (badge) {
      if (this.commandEnabled) {
        badge.textContent = "catch: CMD ON";
        badge.className = "badge bad";
      } else if (this.commandReady) {
        badge.textContent = "catch: ready";
        badge.className = "badge ok";
      } else {
        badge.textContent = "catch: –";
        badge.className = "badge";
      }
    }

    const catchInfo = state.catch;
    if (catchInfo) {
      this.setText("catch-ball", fmtVec(catchInfo.ball_base));
      this.setText("catch-ball-vel", fmtVec(catchInfo.ball_vel_base));
      this.setText("catch-age", fmtMs(catchInfo.perception_age_s, 0));
      this.setText("catch-loop", fmtMs(catchInfo.loop_compute_s, 2));
      this.setText("catch-raw", fmtVec(catchInfo.raw_action));
      this.setText("catch-target", fmtVec(catchInfo.joint_target));
    } else if (!status.alive) {
      for (const id of ["catch-ball", "catch-ball-vel", "catch-age", "catch-loop", "catch-raw", "catch-target"]) {
        this.setText(id, "–");
      }
    }
  }

  refreshButtons() {
    const confirmChecked = document.getElementById("catch-confirm").checked;
    document.getElementById("btn-catch-command-on").disabled =
      !this.commandReady || !confirmChecked || this.commandEnabled;
    document.getElementById("btn-catch-command-off").disabled = !this.commandReady || !this.commandEnabled;
  }

  onBallInputChanged() {
    if (this.mutingInputs) return;
    try {
      this.setBallConfig(this.readBallConfig(), "launch frame edited");
    } catch (error) {
      this.setText("catch-status", error.message);
    }
  }

  onLaunchFrameDragged(config) {
    const next = {
      p0: config.p0 || this.ballConfig.p0,
      v0: config.v0 || this.ballConfig.v0,
      flight_s: this.ballConfig.flight_s,
    };
    const label =
      this.viewer.ballLaunchMode === "aim"
        ? "v0 direction aimed"
        : this.viewer.ballLaunchMode === "speed"
          ? "v0 speed set"
          : "launch frame moved";
    this.setBallConfig(next, label);
  }

  setBallConfig(config, statusText = null) {
    this.ballConfig = {
      p0: [...config.p0],
      v0: [...config.v0],
      flight_s: config.flight_s != null ? config.flight_s : this.ballConfig.flight_s,
    };
    this.mutingInputs = true;
    this.writeBallConfig(this.ballConfig);
    this.viewer.setBallLaunchConfig(this.ballConfig);
    this.mutingInputs = false;
    if (statusText) this.setText("catch-status", statusText);
  }

  readBallConfig() {
    return {
      p0: BALL_FIELD_IDS.p0.map((id) => this.readNumber(id)),
      v0: BALL_FIELD_IDS.v0.map((id) => this.readNumber(id)),
      flight_s: this.readNumber(FLIGHT_FIELD_ID),
    };
  }

  writeBallConfig(config) {
    const write = (ids, values) => {
      ids.forEach((id, index) => {
        document.getElementById(id).value = Number(values[index]).toFixed(3);
      });
    };
    write(BALL_FIELD_IDS.p0, config.p0);
    write(BALL_FIELD_IDS.v0, config.v0);
    if (config.flight_s != null) {
      document.getElementById(FLIGHT_FIELD_ID).value = Number(config.flight_s).toFixed(2);
    }
  }

  readNumber(id) {
    const value = Number.parseFloat(document.getElementById(id).value);
    if (!Number.isFinite(value)) throw new Error(`${id} must be a number`);
    return value;
  }

  setText(id, text) {
    const el = document.getElementById(id);
    if (el) el.textContent = text;
  }
}
