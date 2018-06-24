var util         = require('util');

EKF.DELTA_T = 1 / 15; // In demo mode, 15 navdata per second

module.exports = EKF;
function EKF(options) {

  options = options || {};

  this._options     = options;
  this._delta_t     = options.delta_t || EKF.DELTA_T;

  this.reset();
}

EKF.prototype.state = function() {
    return this._state;
}

EKF.prototype.confidence = function() {
    return this._sigma;
}

EKF.prototype.reset = function() {
  this._state       = this._options.state   || {x: 0, y: 0, yaw: 0};
  this._last_yaw    = null;
}

EKF.prototype.predict = function(data) {
    var pitch = data.demo.rotation.pitch.toRad()
      , roll  = data.demo.rotation.roll.toRad()
      , yaw   = normAngle(data.demo.rotation.yaw.toRad())
      , vx    = data.demo.velocity.x / 1000 //We want m/s instead of mm/s
      , vy    = data.demo.velocity.y / 1000
      , dt    = this._delta_t
    ;

    // We are not interested by the absolute yaw, but the yaw motion,
    // so we need at least a prior value to get started.
    if (this._last_yaw == null) {
        this._last_yaw = yaw;
        return;
    }

    // Compute the odometry by integrating the motion over delta_t
    var o = {dx: vx * dt, dy: vy * dt, dyaw: yaw - this._last_yaw};
    this._last_yaw  = yaw;

    // Update the state estimate
    var state = this._state;
    state.x   = state.x + o.dx * Math.cos(state.yaw) - o.dy * Math.sin(state.yaw);
    state.y   = state.y + o.dx * Math.sin(state.yaw) + o.dy * Math.cos(state.yaw);
    state.yaw = state.yaw + o.dyaw;

    // Normalize the yaw value
    state.yaw = Math.atan2(Math.sin(state.yaw),Math.cos(state.yaw));

}
function normAngle(rad) {
    while (rad >  Math.PI) { rad -= 2 * Math.PI;}
    while (rad < -Math.PI) { rad += 2 * Math.PI;}
    return rad;
}

/** Converts numeric degrees to radians */
if (typeof(Number.prototype.toRad) === "undefined") {
  Number.prototype.toRad = function() {
    return this * Math.PI / 180;
  }
}

/** Converts radians to numeric dregrees */
if (typeof(Number.prototype.toDeg) === "undefined") {
  Number.prototype.toDeg = function() {
    return this * 180 / Math.PI;
  }
}
