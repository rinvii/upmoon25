export type HealthStatus = 'ok' | 'warn' | 'bad' | 'idle'

export type StreamStatus = 'live' | 'connecting' | 'stale' | 'missing' | 'unknown'

export type RobotMode = 'Manual' | 'Assisted' | 'Auto' | 'Paused' | 'Recovery' | 'Estop'

export type MissionState =
  | 'IDLE'
  | 'HEALTH_CHECK'
  | 'WAIT_FOR_ZONE_MARKS'
  | 'READY'
  | 'PERCEPTION_FAULT'
  | 'PERCEPTION_STALE'
  | 'LOCALIZATION_LOST'
  | 'TERRAIN_FAULT'
  | 'TERRAIN_STALE'
  | 'AWAIT_MARK_DIG'
  | 'AWAIT_MARK_DUMP'
  | 'NAV_READY'
  | 'NAV_ACTIVE'
  | 'DISCOVER_DUMP_ZONE'
  | 'NAV_TO_DIG'
  | 'DIG'
  | 'NAV_TO_DUMP'
  | 'DUMP'
  | 'RETURN_TO_DIG'
  | 'RECOVERY'
  | 'PAUSED'
  | 'ABORTED'
  | 'ESTOP'

/** Phases from ``nav_mission_executor`` (JSON field ``phase`` on ``/autonomy/nav_mission/state``). */
export type NavMissionPhase =
  | 'IDLE'
  | 'MAP_EXPLORE'
  | 'HEAD_SWEEP'
  | 'BACKUP_RECOVER'
  | 'FOLLOW_TO_DIG'
  | 'AT_DIG_HANDOFF'
  | 'COMPLETE'
  | 'ABORTED'

export type NavMissionControllerMode = 'idle' | 'mission_twist' | 'corridor_follow' | 'none'

export type NavMissionSnapshot = {
  phase: NavMissionPhase | string
  controllerMode: NavMissionControllerMode | string
  detail?: string
  digAutonomyEnabled?: boolean
  navCorridorEnabled?: boolean
  digDistanceM?: number
  unknownFraction?: number
}

/** Phases from ``dig_sequence`` (JSON ``phase`` on ``/autonomy/dig_sequence/state``). */
export type DigSequencePhase =
  | 'WAIT_NAV_ARM'
  | 'SETUP_IR'
  | 'DRIVE_FORWARD'
  | 'DRIVE_BACK'
  | 'CYCLE_END_CONVEYOR'
  | 'DONE'

export type DigSequenceSnapshot = {
  phase: DigSequencePhase | string
  waitForNavDigArm?: boolean
  digArm?: boolean
  irValue?: number
  irTarget?: number
  encoderValue?: number
  encoderTarget?: number
  encoderTopic?: string
  cycleCounter?: number
  maxCyclesLe?: number
  bucketPosCommanded?: number
  keepBucketChainUntilDone?: boolean
  phaseElapsedSec?: number
  conveyorRemainingSec?: number | null
  /** When true, dig drive phases gate on ``/autonomy/local_terrain_grid`` (same as nav). */
  useLocalTerrainGrid?: boolean
  terrainHadGrid?: boolean
  terrainFresh?: boolean
  terrainForwardOk?: boolean
  terrainReverseOk?: boolean
  terrainGateForward?: string
  terrainGateReverse?: string
}

export type NavigationSteeringSnapshot = {
  linearX: number
  angularZ: number
  reason: string
  ageMs: number | null
}

export type MissionSnapshot = {
  connected: boolean
  armed: boolean
  estop: boolean
  mode: RobotMode
  state: MissionState
  /** When true, bridge published /autonomy/navigation_active for short-segment nav (stack must still gate motion). */
  navigationActive?: boolean
  /** Latest short-segment nav corridor output when navigation stack is running. */
  navigationSteering?: NavigationSteeringSnapshot | null
  /** Start→dig nav mission (optional; live bridge forwards ``/autonomy/nav_mission/state``). */
  navMission?: NavMissionSnapshot | null
  /** ``dig_sequence`` node FSM (optional; live bridge forwards ``/autonomy/dig_sequence/state``). */
  digSequence?: DigSequenceSnapshot | null
  target: string
  heartbeatMs: number | null
  confidence: number
  cycle: number
  payload: 'empty' | 'digging' | 'loaded' | 'dumping' | 'unknown'
  stopReason: string
  lastDecision: string
  nextTransition: string
}

export type Metric = {
  label: string
  value: string
  detail?: string
  status?: HealthStatus
}

export type TopicHealth = {
  topic: string
  status: StreamStatus
  age: string
  rate: string
  note: string
  safetyCritical?: boolean
}

export type CameraStream = {
  id: 'front' | 'rear'
  name: string
  topic: string
  status: StreamStatus
  fps?: string
  resolution?: string
}

export type HardwareRow = {
  category: string
  name: string
  status: string
  detail: string
  severity: HealthStatus
}

export type ZoneStatus = 'unknown' | 'operator_marked' | 'robot_detected' | 'confirmed' | 'stale'

export type FieldZone = {
  id: 'start' | 'dig' | 'dump' | 'no_go'
  label: string
  status: ZoneStatus
  detail: string
  /** Present when operator marked (mission bridge); odom frame. */
  odom_x?: number
  odom_y?: number
}

/** Pose published with operator zone marks (`/autonomy/zone_mark` via mission bridge). */
export type ZoneMarkingSnapshot = {
  poseFrame: string
  odomTopic: string
  odomStatus: StreamStatus
  odomAge: string
  x: number
  y: number
  yawDeg: number | null
}

export type LogLine = {
  ts: string
  level: 'info' | 'warn' | 'error'
  message: string
}

export type TrendPoint = {
  label: string
  cpu: number
  temp: number
  battery: number
  odomVelocity: number
  commandVelocity: number
}

export type SensorSnapshot = {
  irLeft: number | null
  irRight: number | null
  encoderLeft: number | null
  encoderRight: number | null
  encoderPinRight?: number | null
  encoderPinLeft?: number | null
  encoderDecRight?: number | null
  encoderDecLeft?: number | null
  encoderBadRight?: number | null
  encoderBadLeft?: number | null
}

export type ActuatorSnapshot = {
  driveLinear?: number
  driveAngular?: number
  cameraHeight?: number
  panAngle?: number
  bucketPos?: number
  bucketPosMax?: number
  bucketVel?: number
  conveyor?: number
}

export type PidGains = {
  kp: number
  ki: number
  kd: number
}

export type AuditItem = {
  label: string
  status: HealthStatus
  detail: string
}

export type TerrainGridSnapshot = {
  width: number
  height: number
  resolution: number
  frameId: string
  ageMs: number | null
  status: StreamStatus
  cells: number[]
  obstacleCells: number
  cautionCells: number
  unknownCells: number
  note: string
}

export type MissionControlSnapshot = {
  mission: MissionSnapshot
  metrics: Metric[]
  topics: TopicHealth[]
  cameras: CameraStream[]
  hardware: HardwareRow[]
  zones: FieldZone[]
  /** When omitted (older bridge), UI infers odom health from topics/hardware. */
  zoneMarking?: ZoneMarkingSnapshot
  logs: LogLine[]
  trends: TrendPoint[]
  sensors: SensorSnapshot
  actuators?: ActuatorSnapshot
  pid: PidGains
  audit: AuditItem[]
  terrainGrid?: TerrainGridSnapshot
  rawConfig: string
}

export type DriveCommand = 'forward' | 'reverse' | 'left' | 'right' | 'stop'

export type PayloadCommand = 'dig_start' | 'dig_stop' | 'dump_start' | 'dump_stop' | 'stow' | 'abort'

export type ActuatorTarget = 'pan' | 'camera_height' | 'bucket_pos' | 'bucket_vel' | 'conveyor'

export type ActuatorAction = 'increment' | 'decrement' | 'stop' | 'toggle'

export type RobotCommand =
  | { type: 'estop' }
  | { type: 'clear_estop' }
  | { type: 'pause_autonomy' }
  | { type: 'resume_autonomy' }
  | { type: 'manual_takeover' }
  | { type: 'drive'; command: DriveCommand; speedLimit: number }
  | { type: 'payload'; command: PayloadCommand; cycles?: number }
  | { type: 'mark_zone'; zone: FieldZone['id']; pick?: { frameId: 'base_link'; x: number; y: number } }
  | { type: 'set_navigation_active'; active: boolean }
  | { type: 'actuator'; target: ActuatorTarget; action: ActuatorAction; step?: number }
  | { type: 'recording'; enabled: boolean; name?: string }
  | { type: 'save_map'; name: string }
  | { type: 'pid'; gains: PidGains }

export type CommandResult = {
  accepted: boolean
  message: string
  commandId?: string
}
