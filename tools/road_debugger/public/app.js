'use strict';

const $ = (id) => document.getElementById(id);

const elements = {
  dashboard: $('dashboard'),
  connectionBadge: $('connectionBadge'),
  modeBadge: $('modeBadge'),
  packetAge: $('packetAge'),
  leftCount: $('leftCount'),
  centerCount: $('centerCount'),
  rightCount: $('rightCount'),
  frameSequence: $('frameSequence'),
  frameInterval: $('frameInterval'),
  roadCanvas: $('roadCanvas'),
  roadColumnReset: $('roadColumnReset'),
  trendCanvas: $('trendCanvas'),
  scopeLegend: $('scopeLegend'),
  scopeChannelSelect: $('scopeChannelSelect'),
  addScopeChannel: $('addScopeChannel'),
  detectionCanvas: $('detectionCanvas'),
  udpPort: $('udpPort'),
  localAddress: $('localAddress'),
  remoteAddress: $('remoteAddress'),
  jsonPacketCount: $('jsonPacketCount'),
  roadPacketCount: $('roadPacketCount'),
  dropCount: $('dropCount'),
  invalidPacketCount: $('invalidPacketCount'),
  driveState: $('driveState'),
  runIndicator: $('runIndicator'),
  targetBadge: $('targetBadge'),
  itemResult: $('itemResult'),
  redRectText: $('redRectText'),
  plateRectText: $('plateRectText'),
  parameterList: $('parameterList'),
  parameterFilter: $('parameterFilter'),
  recordButton: $('recordButton'),
  recordingName: $('recordingName'),
  recordingState: $('recordingState'),
  recordingSelect: $('recordingSelect'),
  refreshRecordings: $('refreshRecordings'),
  loadRecording: $('loadRecording'),
  returnLiveButton: $('returnLiveButton'),
  playPauseButton: $('playPauseButton'),
  stepBackButton: $('stepBackButton'),
  stepForwardButton: $('stepForwardButton'),
  timeline: $('timeline'),
  playbackTime: $('playbackTime'),
  playbackSpeed: $('playbackSpeed'),
  scopeModeText: $('scopeModeText'),
  parameterTimestamp: $('parameterTimestamp'),
  carIp: $('carIp'),
  carPort: $('carPort'),
  commandInput: $('commandInput'),
  sendCommand: $('sendCommand'),
  commandStatus: $('commandStatus'),
  driveByLeft: $('driveByLeft'),
  driveByRight: $('driveByRight'),
  driveByTestButton: $('driveByTestButton'),
  toast: $('toast'),
};

const PARAMETER_LABELS = {
  seq: '参数包序号',
  uptime_ms: '小车运行时间',
  udp_mode: 'UDP调试模式',
  encoder1_speed_avg: '左轮实际RPS',
  encoder2_speed_avg: '右轮实际RPS',
  latest_error: '视觉误差',
  ex_rps1: '左轮目标RPS',
  ex_rps2: '右轮目标RPS',
  current_pwm1: '左轮PWM',
  current_pwm2: '右轮PWM',
  P1_motor: '左轮P输出',
  P2_motor: '右轮P输出',
  I: '速度环I',
  D1_motor: '左轮I输出',
  D2_motor: '右轮I输出',
  spd_slow_ratio: '减速比例',
  gyro_target_dps: '目标角速度',
  gyro_dps: '实际角速度',
  gyro_timeout: '陀螺仪超时数',
  gyro_read_ms: '陀螺仪读取耗时',
  to_id: '最近超时来源',
  to_used: '最近任务耗时',
  to_target: '任务目标周期',
  to_total: '累计超时数',
  run: '运行状态',
  selected_speed: '速度档位',
  drive_enabled: '识别模式开关',
  drive_busy: '识别流程忙',
  drive_state: '识别状态',
  drive_abort_reason: '绕行退出原因',
  drive_recognizing: '识别阶段',
  drive_motion: '绕行运动阶段',
  drive_test_mode: 'TEST绕行模式',
  drive_brake_active: '主动制动中',
  drive_brake_pwm: '主动制动PWM',
  drive_brake_elapsed_ms: '主动制动耗时',
  drive_test_target_distance_m: 'TEST虚拟目标距离',
  drive_yaw_deg: '绕行累计航向',
  drive_target_yaw_deg: '绕行目标航向',
  drive_heading_error_deg: '绕行航向误差',
  drive_track_heading_deg: '当前赛道航向',
  drive_target_track_heading_deg: '目标处赛道航向',
  drive_view_angle_deg: '目标观察夹角',
  drive_target_distance_m: '触发时目标距离',
  drive_distance_since_trigger_m: '触发后累计距离',
  drive_phase_distance_m: '当前阶段距离',
  drive_target_yaw_rate_dps: '绕行目标角速度',
  drive_turn_rps: '绕行差速输出',
  drive_geometry_valid: '目标几何有效',
  drive_view_ready: '观察角度合格',
  drive_infer_valid_count: '有效识别帧数',
  red_candidate: '远距离红块候选',
  red_candidate_count: '候选连续次数',
  red_contour_area: '红色轮廓面积',
  drive_detection_stage: '目标板检测阶段',
  have_target: '检测到目标',
  item_flag: '识别结果',
  red_x: '红块X',
  red_y: '红块Y',
  red_w: '红块宽',
  red_h: '红块高',
  plate_x: '目标板X',
  plate_y: '目标板Y',
  plate_w: '目标板宽',
  plate_h: '目标板高',
  left_n: '左线点数',
  mid_n: '中线点数',
  right_n: '右线点数',
  circle_type: '环岛状态',
  cross_type: '十字状态',
  track_type: '赛道类型',
  AIM: '前瞻距离',
};

const PARAMETER_ORDER = Object.keys(PARAMETER_LABELS);
const ITEM_NAMES = { 0: '左绕 / 武器', 1: '直行 / 车辆', 2: '右绕 / 物资' };
const SCOPE_CHANNEL_LIMIT = 6;
const SCOPE_COLORS = ['#51d8d0', '#f5df74', '#ff645f', '#5ba9ff', '#ff9c5a', '#b6ef8c'];
const SCOPE_PRESETS = {
  speed: ['encoder1_speed_avg', 'encoder2_speed_avg', 'ex_rps1', 'ex_rps2'],
  gyro: ['gyro_target_dps', 'gyro_dps'],
  drive: ['drive_target_yaw_deg', 'drive_yaw_deg', 'drive_heading_error_deg', 'drive_turn_rps'],
};
const SCOPE_CHANNELS_STORAGE_KEY = 'scopeChannels';
const ROAD_COLUMN_STORAGE_KEY = 'roadColumnPercent';

function loadStoredScopeChannels() {
  try {
    const stored = localStorage.getItem(SCOPE_CHANNELS_STORAGE_KEY);
    if (stored === null) return [...SCOPE_PRESETS.speed];
    const parsed = JSON.parse(stored);
    if (!Array.isArray(parsed)) return [...SCOPE_PRESETS.speed];
    return [...new Set(parsed.filter((key) => typeof key === 'string'))].slice(0, SCOPE_CHANNEL_LIMIT);
  } catch (_error) {
    return [...SCOPE_PRESETS.speed];
  }
}

function loadStoredRoadColumnPercent() {
  const stored = Number(localStorage.getItem(ROAD_COLUMN_STORAGE_KEY));
  return Number.isFinite(stored) && stored >= 30 && stored <= 70 ? stored : 50;
}

const state = {
  mode: 'live',
  liveParams: null,
  liveRoad: null,
  displayParams: null,
  displayRoad: null,
  displayParamTime: null,
  displayRoadTime: null,
  lastPacketAt: 0,
  lastRoadAt: 0,
  previousRoadSequence: null,
  droppedRoadFrames: 0,
  trend: [],
  liveTrend: [],
  serverStatus: null,
  recording: { active: false },
  scopeChannels: loadStoredScopeChannels(),
  roadColumnPercent: loadStoredRoadColumnPercent(),
  driveByTestDirection: 2,
  replay: {
    events: [],
    paramEvents: [],
    roadEvents: [],
    duration: 0,
    currentTime: 0,
    playing: false,
    animationId: 0,
    playWallTime: 0,
    playDataTime: 0,
  },
};

let toastTimer = 0;
let parameterLayoutKey = '';
let parameterRows = new Map();
let scopeOptionsKey = '';

function showToast(message) {
  elements.toast.textContent = message;
  elements.toast.classList.add('visible');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => elements.toast.classList.remove('visible'), 2600);
}

function formatTime(milliseconds) {
  const value = Math.max(0, Number(milliseconds) || 0);
  const minutes = Math.floor(value / 60000);
  const seconds = Math.floor((value % 60000) / 1000);
  const ms = Math.floor(value % 1000);
  return `${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}.${String(ms).padStart(3, '0')}`;
}

function formatValue(key, value) {
  if (value === null || value === undefined) return '--';
  if (typeof value === 'boolean') return value ? '是' : '否';
  if (key === 'run' || key === 'drive_enabled' || key === 'drive_busy' || key === 'have_target' ||
      key === 'drive_recognizing' || key === 'drive_motion' || key === 'drive_geometry_valid' ||
      key === 'drive_view_ready' || key === 'red_candidate' || key === 'drive_test_mode' ||
      key === 'drive_brake_active') {
    return Number(value) ? '是' : '否';
  }
  if (key === 'drive_detection_stage') {
    return ({ 0: '0 / 正常巡线', 1: '1 / 远距接近', 2: '2 / 等待角度',
      3: '3 / 三帧推理', 4: '4 / 绕行运动' })[Number(value)] || String(value);
  }
  if (key === 'item_flag') return `${value} / ${ITEM_NAMES[value] || '未知'}`;
  if (key === 'udp_mode') return ({ 0: '0 / 关闭', 1: '1 / 仅波形', 2: '2 / 波形和道路' })[Number(value)] || String(value);
  if (key === 'uptime_ms') return formatTime(value);
  if (typeof value === 'number' && !Number.isInteger(value)) return value.toFixed(3).replace(/0+$/, '').replace(/\.$/, '');
  return String(value);
}

function parameterDisplayName(key) {
  return PARAMETER_LABELS[key] || key;
}

function numericParameterKeys(params) {
  const keys = Object.keys(params || {}).filter((key) => Number.isFinite(Number(params[key])));
  return [
    ...PARAMETER_ORDER.filter((key) => keys.includes(key)),
    ...keys.filter((key) => !PARAMETER_ORDER.includes(key)).sort(),
  ];
}

function saveScopeChannels() {
  localStorage.setItem(SCOPE_CHANNELS_STORAGE_KEY, JSON.stringify(state.scopeChannels));
}

function renderScopeLegend() {
  if (!state.scopeChannels.length) {
    const hint = document.createElement('span');
    hint.className = 'scope-empty-label';
    hint.textContent = '未选择曲线';
    elements.scopeLegend.replaceChildren(hint);
    return;
  }

  const tags = state.scopeChannels.map((key, index) => {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'scope-channel-tag';
    button.style.setProperty('--channel-color', SCOPE_COLORS[index]);
    button.textContent = parameterDisplayName(key);
    button.title = `${key}；点击删除`;
    button.addEventListener('click', () => {
      setScopeChannels(state.scopeChannels.filter((channel) => channel !== key));
    });
    return button;
  });
  elements.scopeLegend.replaceChildren(...tags);
}

function updateScopeOptions(params) {
  const keys = numericParameterKeys(params);
  state.scopeChannels.forEach((key) => {
    if (!keys.includes(key)) keys.push(key);
  });
  const nextKey = keys.join('\u0000');
  if (nextKey === scopeOptionsKey) return;

  const previousValue = elements.scopeChannelSelect.value;
  const options = keys.map((key) => {
    const option = document.createElement('option');
    option.value = key;
    option.textContent = `${parameterDisplayName(key)} · ${key}`;
    return option;
  });
  if (!options.length) {
    const option = document.createElement('option');
    option.value = '';
    option.textContent = '等待数值型UDP参数';
    options.push(option);
  }
  elements.scopeChannelSelect.replaceChildren(...options);
  if (keys.includes(previousValue)) elements.scopeChannelSelect.value = previousValue;
  scopeOptionsKey = nextKey;
}

function setScopeChannels(channels) {
  const normalized = [...new Set(channels.filter((key) => typeof key === 'string' && key))];
  if (normalized.length > SCOPE_CHANNEL_LIMIT) {
    showToast(`最多同时显示${SCOPE_CHANNEL_LIMIT}条曲线`);
    return;
  }
  state.scopeChannels = normalized;
  saveScopeChannels();
  renderScopeLegend();
  scopeOptionsKey = '';
  updateScopeOptions(state.displayParams || state.liveParams || {});
  drawTrend();
}

function addSelectedScopeChannel() {
  const key = elements.scopeChannelSelect.value;
  if (!key || state.scopeChannels.includes(key)) return;
  setScopeChannels([...state.scopeChannels, key]);
}

function applyRoadColumnPercent(percent, persist = true) {
  const normalized = Math.max(30, Math.min(70, Math.round(percent)));
  state.roadColumnPercent = normalized;
  elements.dashboard.style.setProperty('--road-column-size', `${normalized}fr`);
  elements.dashboard.style.setProperty('--scope-column-size', `${100 - normalized}fr`);
  elements.roadColumnReset.textContent = `图像 ${normalized}%`;
  if (persist) localStorage.setItem(ROAD_COLUMN_STORAGE_KEY, String(normalized));
}

function renderParameters(params) {
  const filter = elements.parameterFilter.value.trim().toLowerCase();
  const keys = params
    ? [...PARAMETER_ORDER.filter((key) => key in params), ...Object.keys(params).filter((key) => !PARAMETER_ORDER.includes(key)).sort()]
    : [];

  const visibleKeys = keys.filter((key) => {
    const label = PARAMETER_LABELS[key] || key;
    return !filter || key.toLowerCase().includes(filter) || label.toLowerCase().includes(filter);
  });
  const layoutKey = `${filter}\u0000${visibleKeys.join('\u0000')}`;

  if (layoutKey !== parameterLayoutKey) {
    parameterRows = new Map();
    const rows = visibleKeys.map((key) => {
      const row = document.createElement('div');
      row.className = 'parameter-row';
      const label = document.createElement('span');
      label.textContent = PARAMETER_LABELS[key] || key;
      label.title = key;
      const value = document.createElement('strong');
      row.append(label, value);
      parameterRows.set(key, value);
      return row;
    });
    elements.parameterList.replaceChildren(...rows);
    parameterLayoutKey = layoutKey;
  }

  visibleKeys.forEach((key) => {
    const value = parameterRows.get(key);
    if (value) value.textContent = formatValue(key, params[key]);
  });
}

function itemResultText(value) {
  const numeric = Number(value);
  return `${Number.isFinite(numeric) ? numeric : 1} / ${ITEM_NAMES[numeric] || '未知'}`;
}

function mergedDetection() {
  const params = state.displayParams || {};
  const road = state.displayRoad;
  const paramRect = (prefix) => ({
    x: Number(params[`${prefix}_x`] || 0),
    y: Number(params[`${prefix}_y`] || 0),
    w: Number(params[`${prefix}_w`] || 0),
    h: Number(params[`${prefix}_h`] || 0),
  });
  return {
    haveTarget: 'have_target' in params ? Boolean(Number(params.have_target)) : Boolean(road?.flags.haveTarget),
    itemFlag: 'item_flag' in params ? Number(params.item_flag) : Number(road?.itemFlag ?? 1),
    redRect: 'red_w' in params ? paramRect('red') : (road?.redRect || { x: 0, y: 0, w: 0, h: 0 }),
    plateRect: 'plate_w' in params ? paramRect('plate') : (road?.plateRect || { x: 0, y: 0, w: 0, h: 0 }),
  };
}

function drawGrid(context, width, height, sourceWidth, sourceHeight) {
  context.fillStyle = '#071113';
  context.fillRect(0, 0, width, height);
  context.lineWidth = 1;
  context.strokeStyle = 'rgba(103, 170, 168, 0.11)';
  for (let x = 0; x <= sourceWidth; x += 40) {
    const px = x / sourceWidth * width;
    context.beginPath();
    context.moveTo(px, 0);
    context.lineTo(px, height);
    context.stroke();
  }
  for (let y = 0; y <= sourceHeight; y += 40) {
    const py = y / sourceHeight * height;
    context.beginPath();
    context.moveTo(0, py);
    context.lineTo(width, py);
    context.stroke();
  }
}

function drawRoad() {
  const canvas = elements.roadCanvas;
  const context = canvas.getContext('2d');
  const road = state.displayRoad;
  const sourceWidth = road?.width || 320;
  const sourceHeight = road?.height || 240;
  drawGrid(context, canvas.width, canvas.height, sourceWidth, sourceHeight);

  const horizon = 100 / sourceHeight * canvas.height;
  context.fillStyle = 'rgba(81, 216, 208, 0.025)';
  context.fillRect(0, horizon, canvas.width, canvas.height - horizon);

  if (!road) {
    context.fillStyle = '#83a09f';
    context.font = '600 25px Bahnschrift, Microsoft YaHei';
    context.textAlign = 'center';
    const hasUdpMode = state.displayParams && 'udp_mode' in state.displayParams;
    const roadEnabled = !hasUdpMode || Number(state.displayParams.udp_mode) >= 2;
    context.fillText(
      roadEnabled ? '等待道路帧 RDL1' : '道路发送已关闭，请发送 #udp=2;',
      canvas.width / 2,
      canvas.height / 2);
    return;
  }

  const scaleX = canvas.width / sourceWidth;
  const scaleY = canvas.height / sourceHeight;
  const drawLine = (points, color, width, glow) => {
    if (!points || points.length < 1) return;
    context.save();
    context.strokeStyle = color;
    context.lineWidth = width;
    context.lineJoin = 'round';
    context.lineCap = 'round';
    context.shadowColor = color;
    context.shadowBlur = glow;
    context.beginPath();
    points.forEach(([x, y], index) => {
      const px = x * scaleX;
      const py = y * scaleY;
      if (index === 0) context.moveTo(px, py);
      else context.lineTo(px, py);
    });
    context.stroke();
    context.restore();
  };

  drawLine(road.lines.left, '#5ba9ff', 5, 8);
  drawLine(road.lines.right, '#ff9c5a', 5, 8);
  drawLine(road.lines.center, '#ecf6f3', 4, 7);

  if (road.aim && road.aim[0] >= 0 && road.aim[1] >= 0) {
    const x = road.aim[0] * scaleX;
    const y = road.aim[1] * scaleY;
    context.save();
    context.strokeStyle = '#ff645f';
    context.lineWidth = 4;
    context.shadowColor = '#ff645f';
    context.shadowBlur = 12;
    context.beginPath();
    context.moveTo(x - 10, y - 10);
    context.lineTo(x + 10, y + 10);
    context.moveTo(x + 10, y - 10);
    context.lineTo(x - 10, y + 10);
    context.stroke();
    context.restore();
  }

  const carX = sourceWidth / 2 * scaleX;
  const carY = (sourceHeight - 11) * scaleY;
  context.fillStyle = '#51d8d0';
  context.beginPath();
  context.moveTo(carX, carY - 18);
  context.lineTo(carX - 13, carY + 12);
  context.lineTo(carX + 13, carY + 12);
  context.closePath();
  context.fill();
}

function drawDetection() {
  const canvas = elements.detectionCanvas;
  const context = canvas.getContext('2d');
  const sourceWidth = 320;
  const sourceHeight = 240;
  drawGrid(context, canvas.width, canvas.height, sourceWidth, sourceHeight);
  const detection = mergedDetection();
  const scaleX = canvas.width / sourceWidth;
  const scaleY = canvas.height / sourceHeight;

  context.save();
  context.setLineDash([8, 8]);
  context.strokeStyle = 'rgba(245, 223, 116, 0.48)';
  context.lineWidth = 2;
  context.strokeRect(60 * scaleX, 110 * scaleY, 200 * scaleX, 90 * scaleY);
  context.restore();

  function drawRect(rect, color, label) {
    if (!rect || rect.w <= 0 || rect.h <= 0) return;
    context.strokeStyle = color;
    context.fillStyle = color;
    context.lineWidth = 4;
    context.strokeRect(rect.x * scaleX, rect.y * scaleY, rect.w * scaleX, rect.h * scaleY);
    context.font = '700 20px Bahnschrift, Microsoft YaHei';
    context.fillText(label, rect.x * scaleX + 4, Math.max(22, rect.y * scaleY - 7));
  }

  drawRect(detection.redRect, '#ff645f', '红块');
  drawRect(detection.plateRect, '#f5df74', '目标板');

  elements.targetBadge.textContent = detection.haveTarget ? '已检测' : '未检测';
  elements.targetBadge.className = `badge ${detection.haveTarget ? 'badge-alert' : 'badge-offline'}`;
  elements.itemResult.textContent = itemResultText(detection.itemFlag);
  const rectText = (rect) => rect.w > 0 && rect.h > 0 ? `(${rect.x}, ${rect.y}) ${rect.w}×${rect.h}` : '--';
  elements.redRectText.textContent = rectText(detection.redRect);
  elements.plateRectText.textContent = rectText(detection.plateRect);
}

function pushTrend(receivedAt, params) {
  state.liveTrend.push({ t: receivedAt, params });
  const cutoff = receivedAt - 10000;
  while (state.liveTrend.length && state.liveTrend[0].t < cutoff) state.liveTrend.shift();
  if (state.mode === 'live') state.trend = state.liveTrend;
}

function buildReplayTrend(currentTime) {
  const events = state.replay.paramEvents;
  state.trend = [];
  if (!events.length) return;
  const cutoff = currentTime - 10000;
  const points = [];
  for (let index = eventIndexAtTime(currentTime, events); index >= 0; index -= 1) {
    const event = events[index];
    if (event.t < cutoff) break;
    points.push({ t: event.t, params: event.data });
  }
  state.trend = points.reverse();
}

function drawTrend() {
  const canvas = elements.trendCanvas;
  const context = canvas.getContext('2d');
  const width = canvas.width;
  const height = canvas.height;
  const plotLeft = 58;
  const plotRight = width - 12;
  const plotTop = 12;
  const plotBottom = height - 20;
  context.fillStyle = '#071113';
  context.fillRect(0, 0, width, height);
  context.strokeStyle = 'rgba(103, 170, 168, 0.12)';
  context.lineWidth = 1;
  for (let index = 0; index <= 5; index += 1) {
    const y = plotTop + index / 5 * (plotBottom - plotTop);
    context.beginPath();
    context.moveTo(plotLeft, y);
    context.lineTo(plotRight, y);
    context.stroke();
  }

  if (!state.scopeChannels.length) {
    context.fillStyle = '#83a09f';
    context.font = '600 18px Bahnschrift, Microsoft YaHei';
    context.textAlign = 'center';
    context.fillText('请添加需要观察的UDP曲线', width / 2, height / 2);
    return;
  }
  if (state.trend.length < 2) return;

  const values = [];
  state.trend.forEach((point) => {
    state.scopeChannels.forEach((key) => {
      const value = Number(point.params?.[key]);
      if (Number.isFinite(value)) values.push(value);
    });
  });
  if (!values.length) return;

  const start = state.trend[0].t;
  const end = Math.max(start + 1, state.trend[state.trend.length - 1].t);
  let axisMin = Math.min(0, ...values);
  let axisMax = Math.max(0, ...values);
  if (axisMin === axisMax) {
    axisMin -= 1;
    axisMax += 1;
  }
  const padding = (axisMax - axisMin) * 0.08;
  axisMin -= padding;
  axisMax += padding;
  const axisRange = axisMax - axisMin;
  const mapY = (value) => plotBottom - (value - axisMin) / axisRange * (plotBottom - plotTop);

  context.fillStyle = '#83a09f';
  context.font = '16px Consolas, monospace';
  context.textAlign = 'right';
  context.textBaseline = 'middle';
  for (let index = 0; index <= 5; index += 1) {
    const y = plotTop + index / 5 * (plotBottom - plotTop);
    const value = axisMax - index / 5 * axisRange;
    context.fillText(value.toFixed(Math.abs(value) < 10 ? 2 : 1), plotLeft - 7, y);
  }

  const zeroY = mapY(0);
  if (zeroY >= plotTop && zeroY <= plotBottom) {
    context.strokeStyle = 'rgba(231, 241, 238, 0.32)';
    context.beginPath();
    context.moveTo(plotLeft, zeroY);
    context.lineTo(plotRight, zeroY);
    context.stroke();
  }

  const drawSeries = (key, color) => {
    context.strokeStyle = color;
    context.lineWidth = 2.5;
    context.beginPath();
    let drawing = false;
    state.trend.forEach((point) => {
      const value = Number(point.params?.[key]);
      if (!Number.isFinite(value)) {
        drawing = false;
        return;
      }
      const x = plotLeft + (point.t - start) / (end - start) * (plotRight - plotLeft);
      const y = mapY(value);
      if (!drawing) context.moveTo(x, y);
      else context.lineTo(x, y);
      drawing = true;
    });
    context.stroke();
  };
  state.scopeChannels.forEach((key, index) => drawSeries(key, SCOPE_COLORS[index]));
}

function updateSummary() {
  const params = state.displayParams || {};
  const road = state.displayRoad;
  const running = Boolean(Number(params.run ?? road?.flags.running ?? 0));
  elements.runIndicator.textContent = running ? 'RUN' : 'STOP';
  elements.runIndicator.classList.toggle('running', running);
  elements.driveState.textContent = params.drive_state || (road?.flags.driveBusy ? 'BUSY' : '--');
  elements.leftCount.textContent = road?.sourceCounts.left ?? params.left_n ?? 0;
  elements.centerCount.textContent = road?.sourceCounts.center ?? params.mid_n ?? 0;
  elements.rightCount.textContent = road?.sourceCounts.right ?? params.right_n ?? 0;
  elements.frameSequence.textContent = road ? `帧 ${road.sequence}` : '帧 --';
  const liveParams = state.liveParams || {};
  const liveRunning = Boolean(Number(liveParams.run ?? 0));
  const driveBusy = Boolean(Number(liveParams.drive_busy ?? 0));
  elements.driveByTestButton.disabled = !liveRunning || driveBusy;
  elements.driveByTestButton.title = !liveRunning
    ? '车辆停车时不可启动绕行测试'
    : (driveBusy ? '绕行脚本正在执行' : '启动绕行脚本测试');
}

function setDriveByTestDirection(direction) {
  state.driveByTestDirection = direction === 0 ? 0 : 2;
  const leftSelected = state.driveByTestDirection === 0;
  elements.driveByLeft.classList.toggle('selected', leftSelected);
  elements.driveByRight.classList.toggle('selected', !leftSelected);
  elements.driveByLeft.setAttribute('aria-pressed', String(leftSelected));
  elements.driveByRight.setAttribute('aria-pressed', String(!leftSelected));
}

function updateScopeModeText() {
  elements.scopeModeText.textContent = state.mode === 'live'
    ? '实时滚动 10 秒'
    : `回放 ${formatTime(state.replay.currentTime)} / ${formatTime(state.replay.duration)}`;
}

function renderDisplaySnapshot() {
  updateScopeOptions(state.displayParams || {});
  renderParameters(state.displayParams || {});
  elements.parameterTimestamp.textContent = state.mode === 'live'
    ? '实时'
    : (state.displayParamTime === null ? '参数 --' : `参数 ${formatTime(state.displayParamTime)}`);
  updateScopeModeText();
  updateSummary();
  drawRoad();
  drawDetection();
}

function applyParams(receivedAt, params) {
  const roadDisabled = 'udp_mode' in params && Number(params.udp_mode) < 2;
  state.liveParams = params;
  state.lastPacketAt = receivedAt;
  pushTrend(receivedAt, params);
  if (roadDisabled) {
    state.liveRoad = null;
    state.previousRoadSequence = null;
  }
  if (state.mode === 'live') {
    state.displayParams = params;
    state.displayParamTime = receivedAt;
    if (roadDisabled) state.displayRoad = null;
    renderDisplaySnapshot();
  }
}

function applyRoad(receivedAt, road) {
  if (state.previousRoadSequence !== null && road.sequence > state.previousRoadSequence + 1) {
    state.droppedRoadFrames += road.sequence - state.previousRoadSequence - 1;
  }
  state.previousRoadSequence = road.sequence;
  if (state.lastRoadAt) elements.frameInterval.textContent = `间隔 ${receivedAt - state.lastRoadAt} ms`;
  state.lastRoadAt = receivedAt;
  state.lastPacketAt = receivedAt;
  state.liveRoad = road;
  if (state.mode === 'live') {
    state.displayRoad = road;
    state.displayRoadTime = receivedAt;
    renderDisplaySnapshot();
  }
}

function applyStatus(status) {
  state.serverStatus = status;
  elements.udpPort.textContent = status.udpPort;
  elements.localAddress.textContent = status.localAddresses?.map((entry) => entry.address).join(' / ') || '--';
  elements.remoteAddress.textContent = status.lastRemote || '--';
  elements.jsonPacketCount.textContent = status.jsonPackets;
  elements.roadPacketCount.textContent = status.roadPackets;
  elements.invalidPacketCount.textContent = status.invalidPackets;
  elements.dropCount.textContent = state.droppedRoadFrames;
  updateRecordingState(status.recording || { active: false });
}

function updateRecordingState(recording) {
  state.recording = recording || { active: false };
  const active = Boolean(recording?.active);
  elements.recordButton.textContent = active ? '停止录制' : '开始录制';
  elements.recordButton.classList.toggle('active', active);
  elements.recordingState.textContent = active
    ? `${recording.filename} / ${recording.eventCount || 0}条 / ${formatTime(recording.durationMs)}`
    : '未录制';
}

function setMode(mode) {
  state.mode = mode;
  elements.modeBadge.textContent = mode === 'live' ? '实时模式' : '回放模式';
  elements.modeBadge.classList.toggle('badge-alert', mode === 'replay');
  if (mode === 'live') {
    pauseReplay();
    state.trend = state.liveTrend;
    state.displayParams = state.liveParams;
    state.displayRoad = state.liveRoad;
    state.displayParamTime = state.lastPacketAt || null;
    state.displayRoadTime = state.lastRoadAt || null;
    renderDisplaySnapshot();
    drawTrend();
  } else {
    updateScopeModeText();
  }
}

function eventIndexAtTime(time, events = state.replay.events) {
  let low = 0;
  let high = events.length;
  while (low < high) {
    const middle = Math.floor((low + high) / 2);
    if (events[middle].t <= time) low = middle + 1;
    else high = middle;
  }
  return low - 1;
}

function applyReplayTime(time) {
  const events = state.replay.events;
  if (!events.length) return;
  const clamped = Math.max(0, Math.min(state.replay.duration, time));
  const paramIndex = eventIndexAtTime(clamped, state.replay.paramEvents);
  const roadIndex = eventIndexAtTime(clamped, state.replay.roadEvents);
  const paramEvent = paramIndex >= 0 ? state.replay.paramEvents[paramIndex] : null;
  const roadEvent = roadIndex >= 0 ? state.replay.roadEvents[roadIndex] : null;
  const roadDisabled = paramEvent && 'udp_mode' in paramEvent.data && Number(paramEvent.data.udp_mode) < 2;

  state.replay.currentTime = clamped;
  state.displayParams = paramEvent?.data || null;
  state.displayRoad = roadDisabled ? null : (roadEvent?.data || null);
  state.displayParamTime = paramEvent?.t ?? null;
  state.displayRoadTime = roadEvent?.t ?? null;
  elements.timeline.value = String(Math.round(clamped));
  elements.playbackTime.textContent = formatTime(clamped);
  elements.frameInterval.textContent = roadEvent
    ? `录像间隔 ${roadIndex > 0 ? roadEvent.t - state.replay.roadEvents[roadIndex - 1].t : '--'} ms`
    : '间隔 -- ms';
  renderDisplaySnapshot();
  buildReplayTrend(clamped);
  drawTrend();
}

function replayTick(now) {
  if (!state.replay.playing) return;
  const speed = Number(elements.playbackSpeed.value || 1);
  const target = state.replay.playDataTime + (now - state.replay.playWallTime) * speed;
  applyReplayTime(target);
  if (target >= state.replay.duration) {
    pauseReplay();
    return;
  }
  state.replay.animationId = requestAnimationFrame(replayTick);
}

function playReplay() {
  if (!state.replay.events.length) {
    showToast('请先载入录像');
    return;
  }
  if (state.replay.currentTime >= state.replay.duration) applyReplayTime(0);
  state.replay.playing = true;
  state.replay.playWallTime = performance.now();
  state.replay.playDataTime = state.replay.currentTime;
  elements.playPauseButton.textContent = 'Ⅱ';
  state.replay.animationId = requestAnimationFrame(replayTick);
}

function pauseReplay() {
  state.replay.playing = false;
  cancelAnimationFrame(state.replay.animationId);
  elements.playPauseButton.textContent = '▶';
}

async function loadRecordings() {
  const response = await fetch('/api/recordings');
  const payload = await response.json();
  elements.recordingSelect.replaceChildren(...(payload.recordings || []).map((recording) => {
    const option = document.createElement('option');
    option.value = recording.name;
    option.textContent = `${recording.name} (${(recording.size / 1024 / 1024).toFixed(1)}MB)`;
    return option;
  }));
  if (!elements.recordingSelect.options.length) {
    const option = document.createElement('option');
    option.textContent = '暂无录像';
    option.value = '';
    elements.recordingSelect.append(option);
  }
}

async function loadSelectedRecording() {
  const name = elements.recordingSelect.value;
  if (!name) {
    showToast('暂无可载入录像');
    return;
  }
  pauseReplay();
  showToast('正在载入录像…');
  const response = await fetch(`/api/recording/file?name=${encodeURIComponent(name)}`);
  if (!response.ok) throw new Error((await response.json()).error || '录像载入失败');
  const text = await response.text();
  const events = text.split(/\r?\n/)
    .filter(Boolean)
    .map((line) => JSON.parse(line))
    .filter((event) => event.type === 'params' || event.type === 'road')
    .sort((left, right) => left.t - right.t);
  if (!events.length) throw new Error('录像中没有有效遥测事件');
  state.replay.events = events;
  state.replay.paramEvents = events.filter((event) => event.type === 'params');
  state.replay.roadEvents = events.filter((event) => event.type === 'road');
  state.replay.duration = events[events.length - 1].t;
  state.replay.currentTime = 0;
  elements.timeline.max = String(Math.max(1, Math.ceil(state.replay.duration)));
  setMode('replay');
  applyReplayTime(0);
  showToast(`已载入 ${events.length} 条事件`);
}

async function toggleRecording() {
  const active = Boolean(state.recording?.active);
  const path = active ? '/api/recording/stop' : '/api/recording/start';
  const response = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name: elements.recordingName.value }),
  });
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || '录制操作失败');
  showToast(active ? '录像已保存' : '开始录制');
  await loadRecordings();
}

async function sendCommand(command) {
  const normalized = String(command || '').trim();
  if (!normalized) return;
  const response = await fetch('/api/command', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      ip: elements.carIp.value,
      port: Number(elements.carPort.value),
      command: normalized,
    }),
  });
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || '发送失败');
  elements.commandStatus.textContent = `已发送 ${payload.command} → ${payload.ip}:${payload.port}`;
  showToast(`已发送 ${payload.command}`);
}

function bindEvents() {
  elements.parameterFilter.addEventListener('input', () => renderParameters(state.displayParams || {}));
  elements.addScopeChannel.addEventListener('click', addSelectedScopeChannel);
  elements.scopeChannelSelect.addEventListener('keydown', (event) => {
    if (event.key === 'Enter') addSelectedScopeChannel();
  });
  document.querySelectorAll('[data-scope-preset]').forEach((button) => {
    button.addEventListener('click', () => setScopeChannels(SCOPE_PRESETS[button.dataset.scopePreset] || []));
  });
  elements.roadColumnReset.addEventListener('click', () => applyRoadColumnPercent(50));
  elements.roadCanvas.addEventListener('wheel', (event) => {
    if (!event.ctrlKey) return;
    event.preventDefault();
    const step = event.deltaY < 0 ? 2 : -2;
    applyRoadColumnPercent(state.roadColumnPercent + step);
  }, { passive: false });
  elements.recordButton.addEventListener('click', () => toggleRecording().catch((error) => showToast(error.message)));
  elements.refreshRecordings.addEventListener('click', () => loadRecordings().catch((error) => showToast(error.message)));
  elements.loadRecording.addEventListener('click', () => loadSelectedRecording().catch((error) => showToast(error.message)));
  elements.returnLiveButton.addEventListener('click', () => setMode('live'));
  elements.playPauseButton.addEventListener('click', () => state.replay.playing ? pauseReplay() : playReplay());
  elements.timeline.addEventListener('input', () => {
    pauseReplay();
    applyReplayTime(Number(elements.timeline.value));
  });
  elements.stepBackButton.addEventListener('click', () => {
    pauseReplay();
    const index = Math.max(0, eventIndexAtTime(state.replay.currentTime) - 1);
    if (state.replay.events[index]) applyReplayTime(state.replay.events[index].t);
  });
  elements.stepForwardButton.addEventListener('click', () => {
    pauseReplay();
    const index = Math.min(state.replay.events.length - 1, eventIndexAtTime(state.replay.currentTime) + 1);
    if (state.replay.events[index]) applyReplayTime(state.replay.events[index].t);
  });
  elements.sendCommand.addEventListener('click', () => sendCommand(elements.commandInput.value).catch((error) => showToast(error.message)));
  elements.commandInput.addEventListener('keydown', (event) => {
    if (event.key === 'Enter') sendCommand(elements.commandInput.value).catch((error) => showToast(error.message));
  });
  document.querySelectorAll('[data-command]').forEach((button) => {
    button.addEventListener('click', () => sendCommand(button.dataset.command).catch((error) => showToast(error.message)));
  });
  elements.driveByLeft.addEventListener('click', () => setDriveByTestDirection(0));
  elements.driveByRight.addEventListener('click', () => setDriveByTestDirection(2));
  elements.driveByTestButton.addEventListener('click', () => {
    sendCommand(`#test_driveby=${state.driveByTestDirection};`).catch((error) => showToast(error.message));
  });
}

function connectEvents() {
  const source = new EventSource('/events');
  source.addEventListener('open', () => {
    elements.connectionBadge.textContent = '服务已连接';
    elements.connectionBadge.className = 'badge badge-online';
  });
  source.addEventListener('error', () => {
    elements.connectionBadge.textContent = '服务重连中';
    elements.connectionBadge.className = 'badge badge-alert';
  });
  source.addEventListener('status', (event) => applyStatus(JSON.parse(event.data)));
  source.addEventListener('recording', (event) => updateRecordingState(JSON.parse(event.data)));
  source.addEventListener('params', (event) => {
    const payload = JSON.parse(event.data);
    applyParams(payload.receivedAt, payload.params);
  });
  source.addEventListener('road', (event) => {
    const payload = JSON.parse(event.data);
    applyRoad(payload.receivedAt, payload.road);
  });
}

function updateConnectionAge() {
  const age = state.lastPacketAt ? Date.now() - state.lastPacketAt : Infinity;
  elements.packetAge.textContent = Number.isFinite(age) ? `${age} ms` : '-- ms';
  if (age < 500) {
    elements.connectionBadge.textContent = 'UDP实时';
    elements.connectionBadge.className = 'badge badge-online';
  } else if (age < 2000) {
    elements.connectionBadge.textContent = 'UDP延迟';
    elements.connectionBadge.className = 'badge badge-alert';
  } else if (state.serverStatus) {
    elements.connectionBadge.textContent = '等待小车UDP';
    elements.connectionBadge.className = 'badge badge-offline';
  }
  elements.dropCount.textContent = state.droppedRoadFrames;
  drawTrend();
}

async function initialize() {
  applyRoadColumnPercent(state.roadColumnPercent, false);
  renderScopeLegend();
  updateScopeOptions({});
  bindEvents();
  drawRoad();
  drawDetection();
  drawTrend();
  connectEvents();
  await loadRecordings().catch(() => null);
  setInterval(updateConnectionAge, 100);
}

initialize();
