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
  driveEnableCommand: $('driveEnableCommand'),
  driveDisableCommand: $('driveDisableCommand'),
  headingHoldCommand: $('headingHoldCommand'),
  tangentDebugCommand: $('tangentDebugCommand'),
  remoteButtons: [...document.querySelectorAll('[data-remote]')],
  tuningControls: $('tuningControls'),
  tuningSnapshotTime: $('tuningSnapshotTime'),
  tuningModeHint: $('tuningModeHint'),
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
  yaw_hold_enabled: '航向保持测试',
  tangent_debug_enabled: '中线切线显示',
  track_tangent_valid: '中线切线有效',
  track_tangent_deg: '中线切线角度',
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
const TUNING_MAXES_STORAGE_KEY = 'tuningSliderMaxes';

// 这里只列出小车端 main.cpp 当前真正支持的在线命令。连续量统一使用
// “滑块 + 数值框”，离散模式使用开关或分段按钮，避免误发无效指令。
const TUNING_GROUPS = [
  {
    title: '速度环',
    subtitle: '车轮转速增量式PID与左右轮前进基准速度',
    controls: [
      { key: 'P', label: '速度P', min: 0, max: 1000, step: 1, defaultValue: 454 },
      { key: 'I', label: '速度I', min: 0, max: 100, step: 0.1, defaultValue: 14 },
      { key: 'D', label: '速度D', min: 0, max: 200, step: 0.1, defaultValue: 0 },
      { key: 'spd', label: '左右轮前进基准速度', min: 0, max: 60, step: 0.5, unit: 'RPS', defaultValue: 0 },
    ],
  },
  {
    title: '视觉方向与道路',
    subtitle: '视觉PD、前瞻与弯道减速',
    controls: [
      { key: 'dirP', label: '方向P', min: 0, max: 2, step: 0.001, defaultValue: 0.128 },
      { key: 'dirD', label: '方向D', min: 0, max: 10, step: 0.01, defaultValue: 1.55 },
      { key: 'AIM', label: '前瞻距离', min: 0.05, max: 1.2, step: 0.01, unit: 'm', defaultValue: 0.25 },
      { key: 'spd_slow_ratio', label: '最大减速比例', min: 0, max: 50, step: 1, unit: '%', defaultValue: 30 },
      { key: 'begin_x', label: '巡线起点X', min: 0, max: 160, step: 1, unit: 'px', defaultValue: 40 },
      { key: 'circle_exit', label: '环岛出环距离', min: 0.1, max: 3, step: 0.01, unit: 'm', defaultValue: 1.2 },
    ],
  },
  {
    title: '角速度环',
    subtitle: '视觉外环与陀螺仪内环',
    controls: [
      { key: 'gyro', label: '角速度反馈', kind: 'toggle', defaultValue: 1 },
      { key: 'gDbg', label: '手动目标模式', kind: 'toggle', defaultValue: 0 },
      { key: 'gTar', label: '手动目标角速度', min: -360, max: 360, step: 1, unit: 'dps', defaultValue: 0 },
      { key: 'gOP', label: '外环P', min: 0, max: 15, step: 0.01, defaultValue: 4.5 },
      { key: 'gOD', label: '外环D', min: 0, max: 10, step: 0.01, defaultValue: 3.3 },
      { key: 'gIP', label: '内环P', min: 0, max: 1.5, step: 0.005, defaultValue: 0.4 },
      { key: 'gII', label: '内环I', min: 0, max: 0.2, step: 0.001, defaultValue: 0 },
      { key: 'gTMax', label: '目标角速度上限', min: 0, max: 720, step: 5, unit: 'dps', defaultValue: 360 },
      { key: 'gRMax', label: '差速上限', min: 0, max: 40, step: 0.5, unit: 'RPS', defaultValue: 20 },
      { key: 'yawHoldRMax', label: '航向保持差速上限', min: 0, max: 40, step: 0.5, unit: 'RPS', defaultValue: 10 },
      { key: 'gSign', label: '陀螺仪符号', kind: 'segment', options: [[-1, '-1'], [1, '+1']], defaultValue: -1 },
      { key: 'tSign', label: '差速输出符号', kind: 'segment', options: [[-1, '-1'], [1, '+1']], defaultValue: 1 },
    ],
  },
  {
    title: '绕行几何',
    subtitle: '方案选择、识别前进速度、距离与航向角外环',
    controls: [
      { key: 'dbMode', label: '绕行方案', kind: 'segment', options: [[0, '角度三阶段'], [1, '边线定时']], defaultValue: 0 },
      { key: 'dbSideMs', label: '边线瞄准持续时间', min: 500, max: 2000, step: 50, unit: 'ms', defaultValue: 500, hardMax: true },
      { key: 'dbUseTangent', label: '目标处切线参考', kind: 'toggle', defaultValue: 0 },
      { key: 'dbNormalSpd', label: '正常巡线前进基准速度', min: 0, max: 60, step: 0.5, unit: 'RPS', defaultValue: 35 },
      { key: 'dbRecSpd', label: '识别阶段前进基准速度', min: 0, max: 40, step: 0.5, unit: 'RPS', defaultValue: 11 },
      { key: 'dbTurnAngle', label: '向外转角', min: 0, max: 90, step: 1, unit: 'deg', defaultValue: 51 },
      { key: 'dbReturnBias', label: '回赛道预偏角', min: 0, max: 91, step: 1, unit: 'deg', defaultValue: 52 },
      { key: 'dbPassDist', label: '最短斜行距离', min: 0, max: 2, step: 0.01, unit: 'm', defaultValue: 0 },
      { key: 'dbSafeDist', label: '目标后安全余量', min: 0, max: 1, step: 0.01, unit: 'm', defaultValue: 0.3 },
      { key: 'dbRpsMps', label: '轮速-车速换算系数', min: 0.01, max: 0.1, step: 0.001, unit: 'm/(s·RPS)', defaultValue: 0.047 },
      { key: 'dbViewMax', label: '最大观察夹角', min: 0, max: 90, step: 1, unit: 'deg', defaultValue: 46 },
      { key: 'dbViewWait', label: '观察等待上限', min: 0, max: 2000, step: 10, unit: 'ms', defaultValue: 120 },
      { key: 'dbHKp', label: '航向外环P', min: 0, max: 60, step: 0.1, defaultValue: 31 },
      { key: 'dbHKd', label: '航向外环D', min: 0, max: 5, step: 0.05, defaultValue: 0.2 },
      { key: 'dbHMax', label: '绕行最大角速度', min: 0, max: 720, step: 5, unit: 'dps', defaultValue: 505 },
      { key: 'dbHTol', label: '航向允许误差（退出+1°）', min: 0, max: 10, step: 0.1, unit: 'deg', defaultValue: 4.5 },
      { key: 'dbRecoverDps', label: '丢线保护角速度', min: 0, max: 360, step: 1, unit: 'dps', defaultValue: 55 },
      { key: 'dbYawSign', label: '绕行航向符号', kind: 'segment', options: [[-1, '-1'], [1, '+1']], defaultValue: -1 },
    ],
  },
  {
    title: '绕行速度与制动',
    subtitle: '运动阶段速度、制动与测试距离',
    controls: [
      { key: 'dbTurnRps', label: '转出阶段前进基准速度', min: 0, max: 40, step: 1, unit: 'RPS', defaultValue: 15 },
      { key: 'dbForwardRps', label: '斜行阶段前进基准速度', min: 0, max: 40, step: 1, unit: 'RPS', defaultValue: 20 },
      { key: 'dbExitRps', label: '转入阶段前进基准速度', min: 0, max: 40, step: 1, unit: 'RPS', defaultValue: 15 },
      { key: 'dbBrakePwm', label: '主动制动反向PWM', min: 0, max: 7000, step: 50, defaultValue: 6000, hardMax: true },
      { key: 'dbBrakeRelease', label: '制动释放速度', min: 0, max: 200, step: 0.5, unit: 'RPS', defaultValue: 15 },
      { key: 'dbBrakeTimeout', label: '制动超时', min: 1, max: 2000, step: 10, unit: 'ms', defaultValue: 300 },
      { key: 'dbTestDist', label: 'TEST目标距离', min: 0, max: 5, step: 0.01, unit: 'm', defaultValue: 0.5 },
    ],
  },
  {
    title: '通信与图像',
    subtitle: 'UDP链路和调试图像模式',
    controls: [
      { key: 'udp', label: 'Debugger UDP', kind: 'segment', options: [[0, '关闭'], [1, '波形'], [2, '波形+三线']], defaultValue: 2 },
      { key: 'vofa', label: '传统VOFA回传', kind: 'toggle', defaultValue: 0 },
      { key: 'is_udp_img', label: 'JPEG调试图像', kind: 'segment', options: [[0, '关闭'], [1, '鸟瞰'], [2, '原图']], defaultValue: 0 },
    ],
  },
  {
    title: '硬件测试',
    subtitle: '仅限run=0：PWM1正向输出，PWM2固定失能',
    controls: [
      { key: 'hwTest', label: 'PWM1硬件测试', kind: 'toggle', defaultValue: 0 },
      { key: 'hwPwm', label: 'PWM1正向占空比', min: 0, max: 5000, step: 50, defaultValue: 0, hardMax: true },
    ],
  },
];

const TUNING_UNIT_GUIDE =
  '先看单位：RPS是车轮每秒转数，表示前进基准速度或左右轮差速，不是车身角速度；' +
  'dps是车身每秒旋转角度。转向时通常为“左轮目标=前进基准RPS+差速RPS，' +
  '右轮目标=前进基准RPS-差速RPS”。deg表示角度，PWM表示直接电机输出。';

const TUNING_DESCRIPTIONS = Object.freeze({
  P: '速度环系数，无直接物理单位。它按本次与上次轮速误差之差增加PWM；调大通常响应更快，过大时轮速容易过冲和振荡。普通速度闭环生效，主动制动和硬件测试时不使用。',
  I: '速度环系数，无直接物理单位。它按当前“目标RPS-实际RPS”持续增加PWM，用来消除稳态速度差；调大能更快追上目标，过大容易过冲。',
  D: '速度环系数，无直接物理单位。它根据连续三次轮速误差的变化修正PWM增量，用于抑制快速变化；调大可能增加阻尼，也会放大编码器噪声。',
  spd: '普通巡线时左右轮共同的前进基准速度，单位RPS，不是角速度。方向控制会在它上面叠加正负差速；设为0只表示没有前进基准，方向环仍可能让两轮反向转动。',
  dirP: '仅用于视觉PD模式或陀螺仪未就绪回退模式。把当前视觉横向误差直接换成轮速差；调大后转向更积极，过大容易在直道左右摇摆。',
  dirD: '仅用于视觉PD模式或回退模式。根据本帧与上帧视觉误差之差生成轮速差；调大可提前抑制转向趋势，但图像噪声大时会产生抖动。',
  AIM: '在鸟瞰中线上向前选取目标点的距离，单位m。调大通常看得更远、行驶更平滑但入弯更晚；调小转弯更灵敏但更容易受近处噪声影响。十字状态可能临时覆盖它。',
  spd_slow_ratio: '视觉误差达到最大值时允许降低的前进基准速度百分比。调大后大弯减速更多、过弯更稳但更慢；0%表示不按视觉误差减速。',
  begin_x: '左右边线搜索在图像底部从“图像中心±该像素值”开始。调大让搜索起点更靠两侧，调小更靠中心；设置不当可能从赛道外或错误边缘开始寻线。',
  circle_exit: '进入环岛运行态后，用里程计累计的出环距离阈值，单位m。调大将在环岛逻辑中保持更久，调小会更早进入出环状态；只对环岛流程生效。',
  gyro: '选择普通方向控制是否请求使用“视觉误差→目标角速度→陀螺仪反馈→轮速差”。关闭时使用视觉PD；开启但MPU6050未就绪时会回退视觉PD。',
  gDbg: '手动目标角速度调试开关。开启后忽略视觉外环，直接用gTar作为目标角速度，并自动请求开启陀螺仪反馈；它不会自动发车。正常行驶应关闭。',
  gTar: '仅在gDbg开启时生效的手动车身目标角速度，单位dps。正负号决定转向方向，绝对值越大要求车身转得越快，并受gTMax限制。',
  gOP: '正常巡线角速度外环P：把视觉误差换成目标角速度dps。调大后同样的道路误差会要求更快转向；过大容易使目标角速度长期顶到gTMax。',
  gOD: '正常巡线角速度外环D：根据相邻两次视觉误差变化修正目标角速度。调大可在误差回落时更早收转向，但会放大图像误差跳变；它不是绕行航向环的dbHKd。',
  gIP: '角速度内环P：把“目标dps-实测dps”换成单侧轮速差RPS。调大后更快追随目标角速度，过大可能造成左右轮反复修正和振荡。正常巡线、绕行和航向保持共用。',
  gII: '角速度内环I：累计目标角速度误差并补充轮速差，用于克服长期左右不一致。调大可减小稳态角速度误差，但容易积累后过冲；陀螺仪数据过旧时积分会冻结。',
  gTMax: '正常视觉外环和手动gTar允许输出的最大车身目标角速度，单位dps。调大只放宽“想转多快”，实际能否达到还受gRMax、速度环和电机能力限制。',
  gRMax: '角速度内环可输出的单侧轮速差上限，单位RPS，不是角速度。轮速通常为基准±差速，所以两轮目标差最多约2倍该值；普通PID_control_test还会再限制到±15RPS。',
  yawHoldRMax: '仅用于停车态“航向保持”测试的额外单侧差速上限，单位RPS，并且仍受gRMax限制。调大回正更有力，但原地拨动车头时动作更猛；不改变正常巡线限幅。',
  gSign: '修正MPU6050实测Z轴角速度的正负方向。若实际向左转时显示符号与控制约定相反，应切换它；设置错误会把负反馈变成错误方向。',
  tSign: '修正角速度内环输出到左右轮差速的方向。若出现“越修越偏”或持续自激旋转，应检查它；它只翻转差速方向，不改变角速度读数。',
  dbMode: '选择下一次绕行采用的脚本：0为陀螺仪航向角三阶段，1为按dbSideMs瞄准绕行侧边线后再恢复中线。绕行开始后会锁存模式，中途修改从下一次生效。',
  dbSideMs: '仅对边线定时方案生效，表示左绕瞄准左线或右绕瞄准右线的持续时间，单位ms。调大后沿边线行驶更久、横向绕得更远；范围固定为500至2000ms，默认500ms。',
  dbUseTangent: '仅对三阶段角度方案生效。开启时用目标位置处识别到的赛道切线作为0度参考；关闭时用脚本启动瞬间车头作为0度参考。切线识别不稳时建议关闭。',
  dbNormalSpd: 'K0绕行识别模式下，无目标以及绕行成功交回巡线后的前进基准速度，单位RPS，不是角速度。方向环仍在此基础上叠加左右差速。',
  dbRecSpd: '红块候选触发后、推理和等待制动期间使用的前进基准速度，单位RPS。它不会强制两轮相等，普通方向环仍叠加差速；调低可减少识别期间前冲距离。',
  dbTurnAngle: '三阶段方案第一次向绕行侧偏转的航向角，单位deg。调大横向移出更明显，但转动时间和占用宽度增加；边线定时方案不使用它。',
  dbReturnBias: '三阶段方案第二次转向的最终目标：相对赛道参考向另一侧预偏该角度，单位deg。第二次总转角约为dbTurnAngle+dbReturnBias；调大更容易朝回赛道，也更可能转过头。',
  dbPassDist: '三阶段斜行阶段必须满足的最小自身行驶距离，单位m，并与“目标距离+安全余量”同时判断。设为0表示只看目标后的安全距离；边线定时方案不使用。',
  dbSafeDist: '三阶段中，车辆估计越过目标板后还需继续前进的安全余量，单位m。调大更晚开始转回、离目标更远，但可能占用更多赛道空间。',
  dbRpsMps: '把编码器平均轮速换算成车速的系数，公式为速度m/s=平均RPS×该值。调大后软件累计距离更快、距离阶段更早结束；应按实测标定，不能用来直接提速。',
  dbViewMax: '目标板允许开始推理的最大观察夹角，单位deg。调大更容易触发但斜视识别风险上升；调小图像更正但可能等待过久。只影响真实红块识别，不影响TEST绕行。',
  dbViewWait: '红块候选成立后，等待观察夹角和ROI满足推理条件的最长时间，单位ms。超时会放弃本次左右绕行并按安全流程退出；调大等待更久也更靠近目标。',
  dbHKp: '三阶段绕行和航向保持的航向角外环P：把航向误差deg换成目标角速度dps。调大起转和回正更积极，过大时更容易顶到dbHMax并冲过目标角。',
  dbHKd: '三阶段绕行和航向保持的航向阻尼：按实测角速度从目标角速度中扣除。调大能更早刹住旋转、减少过冲；过大则转向发软或到角度很慢。',
  dbHMax: '三阶段航向外环允许给出的最大目标角速度，单位dps。调大允许车身转得更快，但实际响应仍受gIP、gII、gRMax、轮速基准和电机能力限制。',
  dbHTol: '三阶段认为航向已进入允许范围的角度阈值，单位deg；进入后角速度还要降下来并连续稳定，退出静默区阈值为该值+1deg。调大更快切阶段但角度精度降低。',
  dbRecoverDps: '绕行回程已经到角度但仍丢中线时，直接请求的固定找线角速度，单位dps。调大找线转得更快，但可能扫过中线；仅在恢复中线保护状态生效。',
  dbYawSign: '把左绕/右绕脚本方向映射到航向角正负号。它影响三阶段目标角的左右对称性，不修改陀螺仪原始读数；左右动作反了时先核对gSign，再核对本项。',
  dbTurnRps: '三阶段TURN_OUT转出时的左右轮前进基准速度，单位RPS，不是角速度。最终轮速=该基准±角速度环差速；调大前进更快，但同样转角需要更强差速和更大空间。',
  dbForwardRps: 'PASS_SHORT斜行阶段的前进基准速度，单位RPS；边线定时方案也使用它。它决定斜行前进快慢，不直接决定车身旋转速度。',
  dbExitRps: '三阶段TURN_TO_TRACK转入及其额外找中线阶段的前进基准速度，单位RPS，不是角速度。调大回赛道更快，但丢线时轨迹更难控制。',
  dbBrakePwm: '红块触发后的主动制动专用反向PWM幅值。调大刹车更强、减速距离更短，但冲击和反转风险更高；最高7000只在主动制动路径允许，正常行驶仍禁止。',
  dbBrakeRelease: '主动制动释放阈值，单位RPS。左右轮连续两次都不高于它就退出反向PWM并恢复速度闭环；调大更早释放，调小刹得更慢、更彻底。',
  dbBrakeTimeout: '主动制动允许持续的最长时间，单位ms。超过后仍未满足释放条件会停车保护；调大不是增强制动力，只是允许反向PWM保持更久。',
  dbTestDist: '点击TEST绕行时模拟的“触发点到目标板”距离，单位m。它参与三阶段通过目标的距离判断；真实红块识别使用视觉估计，不使用该值。',
  udp: '控制发往Debugger（192.168.43.155）的调试数据：0关闭，1只发参数波形，2再加道路左/中/右三线。只控制发送，不影响小车在8082端口接收前端命令。',
  vofa: '控制传统VOFA波形是否回传到192.168.43.146:8080。关闭可减少网络和序列化负担，但仍可从146向小车8082发送调参命令。',
  is_udp_img: '控制额外JPEG调试图像：0关闭，1发送鸟瞰图，2发送原始图。图像编码和网络开销明显高于参数UDP，正常高速行驶建议关闭。',
  hwTest: '仅run=0且绕行、遥控、航向保持都关闭时可开启。开启后只允许PWM1正向测试、PWM2固定为0，并把测试PWM从0开始；关闭状态不会改动正常行车控制。',
  hwPwm: '硬件测试模式下PWM1的正向输出值，范围0至5000，无RPS闭环。关闭测试时修改不会驱动电机，重新开启hwTest仍会先清零，需确认架空车轮后再调。',
});

const TUNING_CONFIGS = TUNING_GROUPS.flatMap((group) => group.controls);
const TUNING_DEFAULTS = Object.fromEntries(
  TUNING_CONFIGS.map((config) => [config.key, config.defaultValue]));

function loadStoredTuningMaxes() {
  try {
    const parsed = JSON.parse(localStorage.getItem(TUNING_MAXES_STORAGE_KEY) || '{}');
    if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed)) return {};
    return Object.fromEntries(Object.entries(parsed)
      .filter(([, value]) => Number.isFinite(Number(value))));
  } catch (_error) {
    return {};
  }
}

const tuningSliderMaxes = loadStoredTuningMaxes();

function tuningSliderMaximum(config, currentValue) {
  const stored = Number(tuningSliderMaxes[config.key]);
  const configured = Number(config.max);
  if (config.hardMax) return configured;
  const minimum = Number(config.min) + Number(config.step);
  let maximum = Number.isFinite(stored) && stored >= minimum ? stored : configured;
  const current = Number(currentValue);
  if (Number.isFinite(current) && current > maximum) maximum = current;
  return maximum;
}

function saveTuningSliderMaxes() {
  try {
    localStorage.setItem(TUNING_MAXES_STORAGE_KEY, JSON.stringify(tuningSliderMaxes));
  } catch (_error) {
    // 浏览器禁用本地存储时仍允许本次页面继续调参。
  }
}

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
  liveTuning: null,
  displayParams: null,
  displayRoad: null,
  displayTuning: null,
  displayParamTime: null,
  displayRoadTime: null,
  displayTuningTime: null,
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
  pendingTuning: new Map(),
  replay: {
    events: [],
    paramEvents: [],
    roadEvents: [],
    tuningEvents: [],
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
const tuningControlElements = new Map();
let remoteHoldDirection = 0;
let remoteHoldButton = null;
let remoteHoldTimer = 0;
let remoteHoldToken = 0;
let remoteHeartbeatInFlight = false;

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
      key === 'drive_brake_active' || key === 'yaw_hold_enabled' ||
      key === 'tangent_debug_enabled' || key === 'track_tangent_valid') {
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

function tuningStepDecimals(step) {
  const text = String(step);
  return text.includes('.') ? text.length - text.indexOf('.') - 1 : 0;
}

function normalizeTuningValue(config, rawValue, maximum = tuningSliderMaximum(config)) {
  let value = Number(rawValue);
  if (!Number.isFinite(value)) value = Number(config.defaultValue);
  if (config.kind === 'toggle') return value ? 1 : 0;
  if (config.kind === 'segment') {
    return config.options.some(([optionValue]) => Number(optionValue) === value)
      ? value
      : Number(config.defaultValue);
  }
  value = Math.max(config.min, Math.min(maximum, value));
  const decimals = tuningStepDecimals(config.step);
  const steps = Math.round((value - config.min) / config.step);
  return Number((config.min + steps * config.step).toFixed(decimals));
}

function commandTuningValue(config, value) {
  if (config.kind === 'toggle' || config.kind === 'segment') return String(Number(value));
  return Number(value).toFixed(tuningStepDecimals(config.step));
}

function tuningValuesForDisplay() {
  return { ...TUNING_DEFAULTS, ...(state.displayTuning || {}) };
}

function renderTuningControls() {
  const values = tuningValuesForDisplay();
  const replaying = state.mode === 'replay';

  for (const [key, control] of tuningControlElements) {
    const maximum = tuningSliderMaximum(control.config, values[key]);
    const value = normalizeTuningValue(control.config, values[key], maximum);
    const editing = document.activeElement === control.range ||
      document.activeElement === control.number ||
      document.activeElement === control.maxInput;
    control.root.classList.toggle('replay-locked', replaying);

    if (control.range) {
      control.range.max = String(maximum);
      control.number.max = String(maximum);
      control.range.disabled = replaying;
      control.number.disabled = replaying;
      if (document.activeElement !== control.maxInput) {
        control.maxInput.value = commandTuningValue(control.config, maximum);
      }
      if (!editing) {
        control.range.value = String(value);
        control.number.value = String(value);
        control.root.classList.remove('has-preview');
      }
    }
    if (control.toggle) {
      control.toggle.disabled = replaying;
      control.toggle.checked = Boolean(Number(value));
    }
    if (control.buttons) {
      control.buttons.forEach((button) => {
        button.disabled = replaying;
        const selected = Number(button.dataset.value) === Number(value);
        button.classList.toggle('selected', selected);
        button.setAttribute('aria-pressed', String(selected));
      });
    }
  }

  if (replaying) {
    elements.tuningSnapshotTime.textContent = state.displayTuningTime === null
      ? '录像无调参快照'
      : `参数 ${formatTime(state.displayTuningTime)}`;
    elements.tuningModeHint.textContent = '回放模式仅显示，不能向小车发送';
  } else {
    elements.tuningSnapshotTime.textContent = state.liveTuning
      ? '小车参数已同步'
      : '等待参数快照';
    elements.tuningModeHint.textContent = '拖动时只预览，松开滑块后发送';
  }
}

async function commitTuningValue(config, rawValue) {
  if (state.mode !== 'live') {
    throw new Error('回放模式不能向小车发送调参指令');
  }

  const value = normalizeTuningValue(config, rawValue);
  const activeControl = tuningControlElements.get(config.key);
  if (activeControl) activeControl.root.classList.remove('has-preview');
  const previousValue = (state.liveTuning && config.key in state.liveTuning)
    ? state.liveTuning[config.key]
    : config.defaultValue;
  state.pendingTuning.set(config.key, { value, sentAt: Date.now() });
  state.liveTuning = { ...TUNING_DEFAULTS, ...(state.liveTuning || {}), [config.key]: value };
  state.displayTuning = state.liveTuning;
  renderTuningControls();

  try {
    await sendCommand(`#${config.key}=${commandTuningValue(config, value)};`, { quiet: true });
  } catch (error) {
    const pending = state.pendingTuning.get(config.key);
    if (pending && Number(pending.value) === Number(value)) {
      state.pendingTuning.delete(config.key);
      state.liveTuning = { ...state.liveTuning, [config.key]: previousValue };
      state.displayTuning = state.liveTuning;
      renderTuningControls();
    }
    throw error;
  }
}

function buildTuningControls() {
  const guide = document.createElement('aside');
  guide.className = 'tuning-unit-guide';
  guide.textContent = TUNING_UNIT_GUIDE;

  const groupElements = TUNING_GROUPS.map((group) => {
    const groupElement = document.createElement('section');
    groupElement.className = 'tuning-group';
    const heading = document.createElement('div');
    heading.className = 'tuning-group-heading';
    const title = document.createElement('h3');
    title.textContent = group.title;
    const subtitle = document.createElement('span');
    subtitle.textContent = group.subtitle;
    heading.append(title, subtitle);
    groupElement.append(heading);

    group.controls.forEach((config) => {
      const root = document.createElement('div');
      root.className = 'tuning-control';
      root.dataset.tuningKey = config.key;
      const labelRow = document.createElement('div');
      labelRow.className = 'tuning-label-row';
      const label = document.createElement('span');
      label.textContent = config.label;
      const code = document.createElement('code');
      code.textContent = config.key;
      labelRow.append(label, code);
      root.append(labelRow);

      const descriptionText = TUNING_DESCRIPTIONS[config.key];
      if (!descriptionText) {
        throw new Error(`缺少调参说明：${config.key}`);
      }
      const description = document.createElement('p');
      description.className = 'tuning-description';
      description.textContent = descriptionText;
      root.append(description);

      const control = { config, root };
      if (config.kind === 'toggle') {
        const toggleLabel = document.createElement('label');
        toggleLabel.className = 'tuning-toggle';
        const toggle = document.createElement('input');
        toggle.type = 'checkbox';
        toggle.setAttribute('aria-label', config.label);
        const track = document.createElement('span');
        track.className = 'tuning-toggle-track';
        const stateText = document.createElement('span');
        stateText.className = 'tuning-toggle-text';
        stateText.textContent = '关闭 / 开启';
        toggleLabel.append(toggle, track, stateText);
        root.append(toggleLabel);
        control.toggle = toggle;
        toggle.addEventListener('change', () => {
          commitTuningValue(config, toggle.checked ? 1 : 0)
            .catch((error) => showToast(error.message));
        });
      } else if (config.kind === 'segment') {
        const segment = document.createElement('div');
        segment.className = 'tuning-segment';
        control.buttons = config.options.map(([value, text]) => {
          const button = document.createElement('button');
          button.type = 'button';
          button.dataset.value = String(value);
          button.textContent = text;
          button.addEventListener('click', () => {
            commitTuningValue(config, value).catch((error) => showToast(error.message));
          });
          segment.append(button);
          return button;
        });
        root.append(segment);
      } else {
        const inputRow = document.createElement('div');
        inputRow.className = 'tuning-input-row';
        const range = document.createElement('input');
        range.type = 'range';
        range.min = String(config.min);
        range.max = String(config.max);
        range.step = String(config.step);
        range.setAttribute('aria-label', `${config.label}滑块`);
        const number = document.createElement('input');
        number.type = 'number';
        number.min = String(config.min);
        const initialMaximum = tuningSliderMaximum(config);
        number.max = String(initialMaximum);
        number.step = String(config.step);
        number.setAttribute('aria-label', `${config.label}数值`);
        const unit = document.createElement('span');
        unit.className = 'tuning-unit';
        unit.textContent = config.unit || '';
        range.max = String(initialMaximum);
        const maxEditor = document.createElement('label');
        maxEditor.className = 'tuning-max-editor';
        const maxLabel = document.createElement('span');
        maxLabel.textContent = '上限';
        const maxInput = document.createElement('input');
        maxInput.type = 'number';
        maxInput.min = String(Number(config.min) + Number(config.step));
        if (config.hardMax) maxInput.max = String(config.max);
        maxInput.step = String(config.step);
        maxInput.value = commandTuningValue(config, initialMaximum);
        maxInput.setAttribute('aria-label', `${config.label}滑块上限`);
        maxEditor.append(maxLabel, maxInput);
        inputRow.append(range, number, unit, maxEditor);
        root.append(inputRow);
        control.range = range;
        control.number = number;
        control.maxInput = maxInput;

        range.addEventListener('input', () => {
          number.value = range.value;
          root.classList.add('has-preview');
        });
        range.addEventListener('change', () => {
          commitTuningValue(config, range.value).catch((error) => showToast(error.message));
        });
        number.addEventListener('input', () => {
          if (Number.isFinite(Number(number.value))) range.value = number.value;
          root.classList.add('has-preview');
        });
        number.addEventListener('change', () => {
          commitTuningValue(config, number.value).catch((error) => showToast(error.message));
        });
        number.addEventListener('keydown', (event) => {
          if (event.key === 'Enter') number.blur();
        });
        maxInput.addEventListener('input', () => {
          const candidate = Number(maxInput.value);
          const current = Number(number.value);
          const minimum = Math.max(Number(config.min) + Number(config.step),
            Number.isFinite(current) ? current : Number(config.min));
          if (Number.isFinite(candidate) && candidate >= minimum) {
            range.max = String(candidate);
            number.max = String(candidate);
          }
        });
        maxInput.addEventListener('change', () => {
          const requested = Number(maxInput.value);
          const current = Number(number.value);
          const minimum = Math.max(Number(config.min) + Number(config.step),
            Number.isFinite(current) ? current : Number(config.min));
          let maximum = Number.isFinite(requested) ? requested : Number(config.max);
          maximum = Math.max(minimum, maximum);
          if (config.hardMax) maximum = Math.min(Number(config.max), maximum);
          const steps = Math.ceil((maximum - Number(config.min)) / Number(config.step));
          maximum = Number((Number(config.min) + steps * Number(config.step))
            .toFixed(tuningStepDecimals(config.step)));
          tuningSliderMaxes[config.key] = maximum;
          saveTuningSliderMaxes();
          range.max = String(maximum);
          number.max = String(maximum);
          maxInput.value = commandTuningValue(config, maximum);
        });
        maxInput.addEventListener('keydown', (event) => {
          if (event.key === 'Enter') maxInput.blur();
        });
      }

      tuningControlElements.set(config.key, control);
      groupElement.append(root);
    });
    return groupElement;
  });

  elements.tuningControls.replaceChildren(guide, ...groupElements);
  renderTuningControls();
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

  if (road.tangent?.valid && road.tangent.anchor?.length === 2) {
    const [anchorX, anchorY] = road.tangent.anchor;
    const angleRad = Number(road.tangent.angleDeg) * Math.PI / 180;
    const directionX = Math.sin(angleRad);
    const directionY = -Math.cos(angleRad);
    const halfLength = 58;
    const startX = (anchorX - directionX * halfLength) * scaleX;
    const startY = (anchorY - directionY * halfLength) * scaleY;
    const endX = (anchorX + directionX * halfLength) * scaleX;
    const endY = (anchorY + directionY * halfLength) * scaleY;

    context.save();
    context.strokeStyle = '#f5df74';
    context.fillStyle = '#f5df74';
    context.lineWidth = 4;
    context.setLineDash([13, 8]);
    context.shadowColor = '#f5df74';
    context.shadowBlur = 9;
    context.beginPath();
    context.moveTo(startX, startY);
    context.lineTo(endX, endY);
    context.stroke();
    context.setLineDash([]);

    // 箭头指向赛道前方，便于区分+角和-角，而不只是看到一条无方向直线。
    const arrowAngle = Math.atan2(endY - startY, endX - startX);
    context.beginPath();
    context.moveTo(endX, endY);
    context.lineTo(
      endX - 15 * Math.cos(arrowAngle - Math.PI / 6),
      endY - 15 * Math.sin(arrowAngle - Math.PI / 6));
    context.lineTo(
      endX - 15 * Math.cos(arrowAngle + Math.PI / 6),
      endY - 15 * Math.sin(arrowAngle + Math.PI / 6));
    context.closePath();
    context.fill();
    context.shadowBlur = 0;
    context.font = '700 20px Bahnschrift, Microsoft YaHei';
    context.fillText(
      `切线 ${Number(road.tangent.angleDeg).toFixed(1)}°`,
      anchorX * scaleX + 12,
      anchorY * scaleY - 12);
    context.restore();
  }

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
  const displayYawHold = Boolean(Number(params.yaw_hold_enabled ?? 0));
  elements.runIndicator.textContent = running ? 'RUN' : (displayYawHold ? 'HOLD' : 'STOP');
  elements.runIndicator.classList.toggle('running', running);
  elements.runIndicator.classList.toggle('holding', !running && displayYawHold);
  elements.driveState.textContent = params.drive_state || (road?.flags.driveBusy ? 'BUSY' : '--');
  elements.leftCount.textContent = road?.sourceCounts.left ?? params.left_n ?? 0;
  elements.centerCount.textContent = road?.sourceCounts.center ?? params.mid_n ?? 0;
  elements.rightCount.textContent = road?.sourceCounts.right ?? params.right_n ?? 0;
  elements.frameSequence.textContent = road ? `帧 ${road.sequence}` : '帧 --';
  const liveParams = state.liveParams || {};
  const liveRunning = Boolean(Number(liveParams.run ?? 0));
  const driveBusy = Boolean(Number(liveParams.drive_busy ?? 0));
  const driveEnabled = Boolean(Number(liveParams.drive_enabled ?? 0));
  const yawHoldEnabled = Boolean(Number(liveParams.yaw_hold_enabled ?? 0));
  const tangentDebugEnabled = Boolean(Number(liveParams.tangent_debug_enabled ?? 0));
  elements.driveByTestButton.disabled = !liveRunning || driveBusy || yawHoldEnabled;
  elements.driveByTestButton.title = !liveRunning
    ? '车辆停车时不可启动绕行测试'
    : (driveBusy ? '绕行脚本正在执行' : '启动绕行脚本测试');
  elements.driveEnableCommand.classList.toggle('selected-command', driveEnabled);
  elements.driveDisableCommand.classList.toggle('selected-command', !driveEnabled);
  elements.headingHoldCommand.classList.toggle('selected-command', yawHoldEnabled);
  elements.headingHoldCommand.textContent = yawHoldEnabled ? '关闭航向保持' : '航向保持测试';
  elements.headingHoldCommand.disabled = state.mode !== 'live' ||
    (!yawHoldEnabled && (liveRunning || driveBusy));
  elements.headingHoldCommand.title = yawHoldEnabled
    ? '关闭停车态航向保持测试'
    : '仅run=0、绕行空闲且陀螺仪正常时可开启';
  elements.tangentDebugCommand.classList.toggle('selected-command', tangentDebugEnabled);
  elements.tangentDebugCommand.textContent = tangentDebugEnabled ? '关闭切线显示' : '中线切线显示';
  elements.tangentDebugCommand.disabled = state.mode !== 'live';
  elements.tangentDebugCommand.title = '开启时自动切到UDP+道路三线';

  const remoteEnabled = state.mode === 'live' && Boolean(state.liveParams) &&
    !liveRunning && !yawHoldEnabled;
  elements.remoteButtons.forEach((button) => {
    button.disabled = !remoteEnabled;
    button.title = remoteEnabled ? '按住移动，松开立即停止' : '仅实时模式且run=0时可用';
  });
  if (!remoteEnabled && remoteHoldDirection !== 0) {
    stopRemoteHold();
  }
}

async function toggleHeadingHold() {
  const enabled = Boolean(Number(state.liveParams?.yaw_hold_enabled ?? 0));
  if (!enabled && Number(state.liveParams?.udp_mode ?? 0) === 0) {
    await sendCommand('#udp=1;', { quiet: true });
  }
  await sendCommand(`#yawHold=${enabled ? 0 : 1};`);
}

async function toggleTangentDebug() {
  const enabled = Boolean(Number(state.liveParams?.tangent_debug_enabled ?? 0));
  if (!enabled && Number(state.liveParams?.udp_mode ?? 0) < 2) {
    await sendCommand('#udp=2;', { quiet: true });
  }
  await sendCommand(`#tangentDbg=${enabled ? 0 : 1};`);
}

async function sendRemoteHeartbeat(token) {
  if (token !== remoteHoldToken || remoteHoldDirection === 0 || remoteHeartbeatInFlight) return;
  remoteHeartbeatInFlight = true;
  try {
    await sendCommand(`#remote=${remoteHoldDirection};`, { quiet: true, silentStatus: true });
  } catch (error) {
    showToast(`遥控发送失败：${error.message}`);
    stopRemoteHold();
  } finally {
    remoteHeartbeatInFlight = false;
    // 方向包发送期间如果已经松开，再补发一次停止，避免网络乱序留下旧方向。
    if (token !== remoteHoldToken || remoteHoldDirection === 0) {
      sendCommand('#remote=0;', { quiet: true, silentStatus: true }).catch(() => {});
    }
  }
}

function startRemoteHold(direction, button) {
  const liveRunning = Boolean(Number(state.liveParams?.run ?? 0));
  if (state.mode !== 'live' || !state.liveParams || liveRunning || button.disabled) return;

  stopRemoteHold(false);
  remoteHoldDirection = direction;
  remoteHoldButton = button;
  remoteHoldButton.classList.add('remote-active');
  remoteHoldToken += 1;
  const token = remoteHoldToken;
  elements.commandStatus.textContent = `停车遥控中：${button.textContent.trim()}`;
  sendRemoteHeartbeat(token);
  remoteHoldTimer = window.setInterval(() => sendRemoteHeartbeat(token), 100);
}

function stopRemoteHold(sendStop = true) {
  if (remoteHoldTimer) window.clearInterval(remoteHoldTimer);
  remoteHoldTimer = 0;
  remoteHoldDirection = 0;
  remoteHoldToken += 1;
  if (remoteHoldButton) remoteHoldButton.classList.remove('remote-active');
  remoteHoldButton = null;
  if (sendStop) {
    sendCommand('#remote=0;', { quiet: true, silentStatus: true }).catch(() => {});
    elements.commandStatus.textContent = '停车遥控已停止';
  }
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
  renderTuningControls();
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

function applyTuning(receivedAt, tuning) {
  const merged = { ...TUNING_DEFAULTS, ...tuning };
  const now = Date.now();

  // 指令发出到下一份250ms快照之间保留前端预期值。快照确认一致后清除，
  // 若1.5秒仍未确认则以小车实际回传为准，避免界面长期显示假状态。
  for (const [key, pending] of state.pendingTuning) {
    if (Number(merged[key]) === Number(pending.value)) {
      state.pendingTuning.delete(key);
    } else if (now - pending.sentAt < 1500) {
      merged[key] = pending.value;
    } else {
      state.pendingTuning.delete(key);
    }
  }

  state.liveTuning = merged;
  state.lastPacketAt = receivedAt;
  if (state.mode === 'live') {
    state.displayTuning = merged;
    state.displayTuningTime = receivedAt;
    renderTuningControls();
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
    state.displayTuning = state.liveTuning;
    state.displayParamTime = state.lastPacketAt || null;
    state.displayRoadTime = state.lastRoadAt || null;
    state.displayTuningTime = state.liveTuning ? state.lastPacketAt : null;
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
  const tuningIndex = eventIndexAtTime(clamped, state.replay.tuningEvents);
  const paramEvent = paramIndex >= 0 ? state.replay.paramEvents[paramIndex] : null;
  const roadEvent = roadIndex >= 0 ? state.replay.roadEvents[roadIndex] : null;
  const tuningEvent = tuningIndex >= 0 ? state.replay.tuningEvents[tuningIndex] : null;
  const roadDisabled = paramEvent && 'udp_mode' in paramEvent.data && Number(paramEvent.data.udp_mode) < 2;

  state.replay.currentTime = clamped;
  state.displayParams = paramEvent?.data || null;
  state.displayRoad = roadDisabled ? null : (roadEvent?.data || null);
  state.displayTuning = tuningEvent?.data || null;
  state.displayParamTime = paramEvent?.t ?? null;
  state.displayRoadTime = roadEvent?.t ?? null;
  state.displayTuningTime = tuningEvent?.t ?? null;
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
    .filter((event) => event.type === 'params' || event.type === 'road' || event.type === 'tuning')
    .sort((left, right) => left.t - right.t);
  if (!events.length) throw new Error('录像中没有有效遥测事件');
  state.replay.events = events;
  state.replay.paramEvents = events.filter((event) => event.type === 'params');
  state.replay.roadEvents = events.filter((event) => event.type === 'road');
  state.replay.tuningEvents = events.filter((event) => event.type === 'tuning');
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

async function sendCommand(command, options = {}) {
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
  if (!options.silentStatus) {
    elements.commandStatus.textContent = `已发送 ${payload.command} → ${payload.ip}:${payload.port}`;
  }
  if (!options.quiet) showToast(`已发送 ${payload.command}`);
  return payload;
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
  elements.headingHoldCommand.addEventListener('click', () => {
    toggleHeadingHold().catch((error) => showToast(error.message));
  });
  elements.tangentDebugCommand.addEventListener('click', () => {
    toggleTangentDebug().catch((error) => showToast(error.message));
  });
  elements.remoteButtons.forEach((button) => {
    button.addEventListener('contextmenu', (event) => event.preventDefault());
    button.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      button.setPointerCapture?.(event.pointerId);
      startRemoteHold(Number(button.dataset.remote), button);
    });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((eventName) => {
      button.addEventListener(eventName, () => {
        if (remoteHoldButton === button) stopRemoteHold();
      });
    });
  });
  window.addEventListener('blur', () => {
    if (remoteHoldDirection !== 0) stopRemoteHold();
  });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden && remoteHoldDirection !== 0) stopRemoteHold();
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
  source.addEventListener('tuning', (event) => {
    const payload = JSON.parse(event.data);
    applyTuning(payload.receivedAt, payload.tuning);
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
  buildTuningControls();
  bindEvents();
  drawRoad();
  drawDetection();
  drawTrend();
  connectEvents();
  await loadRecordings().catch(() => null);
  setInterval(updateConnectionAge, 100);
}

initialize();
