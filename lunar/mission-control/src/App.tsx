import { memo, useEffect, useMemo, useRef, useState } from 'react'
import {
  Activity,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  ArrowUp,
  BarChart3,
  Camera,
  CircleStop,
  Cpu,
  Database,
  Flag,
  Gauge,
  GitBranch,
  Home,
  MapPinned,
  Pause,
  Play,
  Power,
  Radio,
  Save,
  ShieldAlert,
  Shovel,
  SlidersHorizontal,
  Square,
  Terminal,
  Video,
  WifiOff,
} from 'lucide-react'
import { clsx } from 'clsx'
import type {
  CameraStream,
  FieldZone,
  HealthStatus,
  Metric,
  MissionControlSnapshot,
  RobotCommand,
  StreamStatus,
  TerrainGridSnapshot,
  ZoneMarkingSnapshot,
} from './bridge/types'
import type { BridgeMode } from './bridge/bridgeCapabilities'
import { commandButtonState } from './bridge/commandGate'
import type { DemoScenario } from './bridge/mockSnapshot'
import { useMissionBridge } from './bridge/useMissionBridge'
import { DigDumpMissionChart } from './components/DigDumpMissionChart'
import { DigSequenceStateChart } from './components/DigSequenceStateChart'
import { NavAutonomyStateChart } from './components/NavAutonomyStateChart'
import { ZoneMarkingPanel } from './components/ZoneMarkingPanel'
import { useCameraFrame } from './hooks/useCameraFrame'
import { useCameraWsBaseUrl } from './hooks/useCameraWsBaseUrl'
import { useSensorWebSocket } from './hooks/useSensorWebSocket'

const scenarioLabels: Record<DemoScenario, string> = {
  nominal: 'Nominal',
  degraded: 'Degraded',
  offline: 'Offline',
  terrain_lab: 'Terrain lab',
}

function isNominalLikeScenario(s: DemoScenario): boolean {
  return s === 'nominal' || s === 'terrain_lab'
}

function steeringMetric(snapshot: MissionControlSnapshot): Metric {
  const nav = snapshot.mission.navigationSteering
  if (nav) {
    const moving = Math.abs(nav.linearX) > 0.01 || Math.abs(nav.angularZ) > 0.01
    const age = nav.ageMs === null ? '' : ` · ${nav.ageMs} ms`
    return {
      label: 'Steering',
      value: moving ? `${nav.linearX.toFixed(2)} m/s` : 'hold',
      detail: `${nav.reason || 'navigation'}${age}`,
      status: moving ? 'ok' : 'idle',
    }
  }
  return {
    label: 'Steering',
    value: '--',
    detail: '/autonomy/navigation_status',
    status: 'idle',
  }
}

function initialDemoScenario(): DemoScenario {
  if (typeof window === 'undefined') return 'degraded'
  const raw = new URLSearchParams(window.location.search).get('scenario')
  const allowed: DemoScenario[] = ['nominal', 'degraded', 'offline', 'terrain_lab']
  if (raw && (allowed as readonly string[]).includes(raw)) return raw as DemoScenario
  return 'degraded'
}

function statusClass(status: HealthStatus = 'idle') {
  return {
    ok: 'border-emerald-500/40 bg-emerald-500/10 text-emerald-200',
    warn: 'border-amber-500/40 bg-amber-500/10 text-amber-100',
    bad: 'border-red-500/50 bg-red-500/10 text-red-100',
    idle: 'border-slate-700 bg-slate-900/70 text-slate-200',
  }[status]
}

function streamToStatus(status: StreamStatus): HealthStatus {
  const statuses: Record<StreamStatus, HealthStatus> = {
    live: 'ok',
    connecting: 'warn',
    stale: 'warn',
    missing: 'bad',
    unknown: 'idle',
  }
  return statuses[status]
}

function Panel({
  title,
  icon,
  children,
  className,
  action,
}: {
  title: string
  icon?: React.ReactNode
  children: React.ReactNode
  className?: string
  action?: React.ReactNode
}) {
  return (
    <section className={clsx('rounded-lg border border-slate-800 bg-slate-950/80 shadow-xl shadow-black/10', className)}>
      <div className="flex min-h-11 items-center justify-between gap-3 border-b border-slate-800 px-4 py-2">
        <div className="flex items-center gap-2 text-sm font-semibold text-slate-100">
          {icon}
          <span>{title}</span>
        </div>
        {action}
      </div>
      <div className="p-4">{children}</div>
    </section>
  )
}

function Button({
  children,
  intent = 'default',
  className,
  disabled,
  title,
  onClick,
}: {
  children: React.ReactNode
  intent?: 'default' | 'danger' | 'safe' | 'warn'
  className?: string
  disabled?: boolean
  title?: string
  onClick?: () => void
}) {
  const styles = {
    default: 'border-slate-700 bg-slate-900 text-slate-100 hover:bg-slate-800',
    danger: 'border-red-500/60 bg-red-600 text-white hover:bg-red-500',
    safe: 'border-emerald-500/50 bg-emerald-600/90 text-white hover:bg-emerald-500',
    warn: 'border-amber-500/50 bg-amber-500/90 text-slate-950 hover:bg-amber-400',
  }
  return (
    <button
      type="button"
      disabled={disabled}
      title={title}
      onClick={onClick}
      className={clsx(
        'inline-flex min-h-9 items-center justify-center gap-2 rounded-md border px-3 py-1.5 text-sm font-semibold transition disabled:cursor-not-allowed disabled:opacity-45',
        styles[intent],
        className,
      )}
    >
      {children}
    </button>
  )
}

function MetricCard({ metric }: { metric: Metric }) {
  return (
    <div className={clsx('min-w-0 rounded-md border p-3', statusClass(metric.status))}>
      <div className="text-[11px] uppercase tracking-wide opacity-75">{metric.label}</div>
      <div className="mt-1 break-words text-xl font-semibold text-white">{metric.value}</div>
      {metric.detail ? <div className="mt-1 break-words text-xs opacity-75">{metric.detail}</div> : null}
    </div>
  )
}

function AlertDot({ status }: { status: HealthStatus }) {
  return <span className={clsx('h-2.5 w-2.5 rounded-full', status === 'ok' ? 'bg-emerald-300' : status === 'warn' ? 'bg-amber-300' : status === 'bad' ? 'bg-red-300' : 'bg-slate-500')} />
}

function EmptyState({ title, detail, status = 'idle' }: { title: string; detail: string; status?: HealthStatus }) {
  return (
    <div className={clsx('rounded-md border p-3 text-sm', statusClass(status))}>
      <div className="flex items-center gap-2 font-semibold text-white">
        {status === 'bad' ? <WifiOff className="h-4 w-4" /> : <AlertDot status={status} />}
        {title}
      </div>
      <div className="mt-1 text-xs opacity-80">{detail}</div>
    </div>
  )
}

function SafetyBar({
  snapshot,
  scenario,
  setScenario,
  bridgeMode,
  setBridgeMode,
  liveAvailable,
  runCommand,
}: {
  snapshot: MissionControlSnapshot
  scenario: DemoScenario
  setScenario: (scenario: DemoScenario) => void
  bridgeMode: BridgeMode
  setBridgeMode: (mode: BridgeMode) => void
  liveAvailable: boolean
  runCommand: (command: RobotCommand) => Promise<void>
}) {
  const { mission } = snapshot
  const navOn = Boolean(mission.navigationActive)
  const connected = mission.connected
  const navToggle = commandButtonState(bridgeMode, connected, { type: 'set_navigation_active', active: !navOn })
  const estopBtn = commandButtonState(bridgeMode, connected, { type: 'estop' })
  const pauseBtn = commandButtonState(bridgeMode, connected, { type: 'pause_autonomy' })
  const resumeBtn = commandButtonState(bridgeMode, connected, { type: 'resume_autonomy' }, mission.estop)
  const manualBtn = commandButtonState(bridgeMode, connected, { type: 'manual_takeover' })
  const clearEstopBtn = commandButtonState(bridgeMode, connected, { type: 'clear_estop' })
  return (
    <div className="sticky top-0 z-50 border-b border-slate-800 bg-slate-950/95 px-3 py-3 backdrop-blur sm:px-4">
      <div className="mx-auto flex max-w-[1800px] flex-wrap items-center gap-3">
        <div className="flex items-center gap-2 text-sm font-semibold text-slate-100">
          <ShieldAlert className="h-5 w-5 text-amber-300" />
          Lunar Mission Control
        </div>
        <div className="flex flex-wrap items-center gap-2 text-xs">
          <span className={clsx('rounded-full border px-2 py-1', statusClass(mission.connected ? 'ok' : 'bad'))}>Bridge {mission.connected ? 'online' : 'offline'}</span>
          <span className={clsx('rounded-full border px-2 py-1', statusClass(mission.armed ? 'warn' : 'idle'))}>{mission.armed ? 'Armed' : 'Disarmed'}</span>
          <span className={clsx('rounded-full border px-2 py-1', statusClass(mission.estop ? 'bad' : 'ok'))}>E-stop {mission.estop ? 'active' : 'clear'}</span>
          <span className="rounded-full border border-slate-700 bg-slate-900 px-2 py-1">Mode {mission.mode}</span>
          <span className="rounded-full border border-slate-700 bg-slate-900 px-2 py-1">State {mission.state}</span>
          <span
            className={clsx(
              'rounded-full border px-2 py-1',
              statusClass(navOn ? 'warn' : 'idle'),
            )}
          >
            Short nav {navOn ? 'armed' : 'off'}
          </span>
          {mission.navMission?.phase ? (
            <span className="rounded-full border border-violet-800 bg-violet-950/60 px-2 py-1 font-mono text-violet-100">
              Nav mission {mission.navMission.phase}
            </span>
          ) : null}
          {mission.digSequence?.phase ? (
            <span className="rounded-full border border-amber-800/80 bg-amber-950/50 px-2 py-1 font-mono text-amber-100">
              Dig seq {mission.digSequence.phase}
            </span>
          ) : null}
          <span className="rounded-full border border-slate-700 bg-slate-900 px-2 py-1">Heartbeat {mission.heartbeatMs === null ? '--' : `${mission.heartbeatMs} ms`}</span>
        </div>
        <select
          value={scenario}
          onChange={(event) => setScenario(event.target.value as DemoScenario)}
          disabled={bridgeMode === 'live'}
          className="rounded-md border border-slate-700 bg-slate-900 px-2 py-1 text-xs text-slate-100"
          aria-label="Demo scenario"
        >
          {Object.entries(scenarioLabels).map(([value, label]) => (
            <option key={value} value={value}>
              {label}
            </option>
          ))}
        </select>
        <select
          value={bridgeMode}
          onChange={(event) => setBridgeMode(event.target.value as 'mock' | 'live')}
          className="rounded-md border border-slate-700 bg-slate-900 px-2 py-1 text-xs text-slate-100"
          aria-label="Bridge mode"
        >
          <option value="mock">Mock bridge</option>
          <option value="live" disabled={!liveAvailable}>Live bridge</option>
        </select>
        <div className="ml-auto flex flex-wrap items-center gap-2">
          <Button
            intent="safe"
            disabled={navToggle.disabled}
            title={navToggle.title}
            className="min-w-32"
            onClick={() => void runCommand({ type: 'set_navigation_active', active: !navOn })}
          >
            <Radio className="h-4 w-4" /> Short nav {navOn ? 'off' : 'on'}
          </Button>
          <Button
            intent="danger"
            className="min-w-28"
            disabled={estopBtn.disabled}
            title={estopBtn.title}
            onClick={() => void runCommand({ type: 'estop' })}
          >
            <Power className="h-4 w-4" /> ESTOP
          </Button>
          {mission.estop ? (
            <Button
              intent="warn"
              className="min-w-32"
              disabled={clearEstopBtn.disabled}
              title={clearEstopBtn.title ?? 'Clear dashboard ESTOP after physical safety check'}
              onClick={() => void runCommand({ type: 'clear_estop' })}
            >
              Clear ESTOP
            </Button>
          ) : null}
          <Button
            intent="warn"
            disabled={pauseBtn.disabled}
            title={pauseBtn.title}
            onClick={() => void runCommand({ type: 'pause_autonomy' })}
          >
            <Pause className="h-4 w-4" /> Pause
          </Button>
          <Button
            intent="safe"
            disabled={resumeBtn.disabled}
            title={resumeBtn.title ?? (mission.estop ? 'Clear ESTOP before resume' : undefined)}
            onClick={() => void runCommand({ type: 'resume_autonomy' })}
          >
            <Play className="h-4 w-4" /> Resume
          </Button>
          <Button
            disabled={manualBtn.disabled}
            title={manualBtn.title}
            onClick={() => void runCommand({ type: 'manual_takeover' })}
          >
            <CircleStop className="h-4 w-4" /> Manual Takeover
          </Button>
        </div>
      </div>
    </div>
  )
}

const CameraFeed = memo(function CameraFeed({ camera, wsBase }: { camera: CameraStream; wsBase: string | undefined }) {
  const { imageRef, hasFrame, socketState, configured } = useCameraFrame(camera, wsBase)
  const tryWs = Boolean(configured)
  const displayStatus = tryWs ? socketState : camera.status
  const activeFrame = tryWs && socketState === 'live' && hasFrame
  const statusText = {
    live: 'live',
    connecting: 'connecting',
    stale: 'stale frame',
    missing: 'no stream',
    unknown: 'unknown',
  }[displayStatus]
  const detail = {
    live: activeFrame ? 'Receiving JPEG frames from camera_ws (Tornado)' : 'WebSocket open; waiting for first JPEG',
    connecting: 'Connecting to camera WebSocket (auto-retry every 500 ms)',
    stale: 'WebSocket error; retrying',
    missing: configured ? 'Camera WebSocket closed or unreachable; retrying' : 'Could not resolve WebSocket base URL for this page',
    unknown: 'Camera has not been checked in this session',
  }[displayStatus]
  const accent = camera.id === 'front' ? 'border-orange-300' : camera.id === 'rear' ? 'border-sky-300' : 'border-slate-400'
  const imageStyle = camera.id === 'rear' ? { transform: 'rotate(180deg)' } : undefined

  return (
    <div className="overflow-hidden rounded-md border border-slate-800 bg-slate-900">
      <div className="relative aspect-video bg-[radial-gradient(circle_at_40%_35%,#334155,#0f172a_42%,#020617)]">
        <img
          ref={imageRef}
          className={clsx('h-full w-full object-cover', activeFrame ? 'opacity-100' : 'opacity-0')}
          style={imageStyle}
          alt={`${camera.name} live camera feed`}
        />
        {!activeFrame && displayStatus === 'live' ? (
          <>
            <div className={clsx('absolute left-1/2 top-1/2 h-16 w-16 -translate-x-1/2 -translate-y-1/2 rounded-full border-2', accent)} />
            <div className="absolute bottom-4 left-4 right-4 h-px bg-white/30" />
            <div className="absolute bottom-4 left-1/2 top-4 w-px bg-white/30" />
          </>
        ) : !activeFrame ? (
          <div className="absolute inset-0 flex items-center justify-center p-4">
            <div className="max-w-xs rounded-md border border-slate-700 bg-black/60 p-4 text-center">
              <div className="text-sm font-semibold text-white">{statusText}</div>
              <div className="mt-1 text-xs text-slate-300">{detail}</div>
            </div>
          </div>
        ) : null}
        <div className={clsx('absolute left-4 top-4 rounded border px-2 py-1 text-xs', statusClass(streamToStatus(displayStatus)))}>{statusText}</div>
      </div>
      <div className="flex items-center justify-between gap-2 px-3 py-2 text-xs text-slate-300">
        <span>{camera.name}</span>
        <span>{camera.fps ?? '--'} FPS | {camera.resolution ?? '--'}</span>
      </div>
      <div className="border-t border-slate-800 px-3 py-2 font-mono text-[11px] text-slate-500">{camera.topic}</div>
    </div>
  )
})

const TERRAIN_PIXEL: Record<'ok' | 'warn' | 'bad' | 'unknown' | 'robot', string> = {
  /** Free / traversable */
  ok: 'rgba(22, 163, 74, 0.92)',
  /** Uneven / drop-off risk — backend cell value 60 (not free, not a hard obstacle) */
  warn: 'rgba(234, 179, 8, 0.9)',
  /** Obstacle (backend ≥90) */
  bad: 'rgba(185, 28, 28, 0.94)',
  /** Unknown / not yet classified (backend -1) */
  unknown: 'rgba(120, 120, 120, 0.92)',
  /** Robot forward strip */
  robot: 'rgba(255, 255, 255, 0.96)',
}

const ZONE_MARKER_RGBA: Record<FieldZone['id'], string> = {
  start: 'rgba(52, 211, 153, 0.95)',
  dig: 'rgba(250, 204, 21, 0.98)',
  dump: 'rgba(249, 115, 22, 0.98)',
  no_go: 'rgba(220, 38, 38, 0.98)',
}

function odomToTerrainCell(
  robotX: number,
  robotY: number,
  yawDeg: number,
  worldX: number,
  worldY: number,
  width: number,
  height: number,
  resolutionM: number,
): { ix: number; iy: number } | null {
  const yaw = (yawDeg * Math.PI) / 180
  const dx = worldX - robotX
  const dy = worldY - robotY
  const lx = Math.cos(yaw) * dx + Math.sin(yaw) * dy
  const ly = -Math.sin(yaw) * dx + Math.cos(yaw) * dy
  const widthM = width * resolutionM
  const iy = Math.min(height - 1, Math.max(0, Math.floor(lx / resolutionM)))
  const ix = Math.min(width - 1, Math.max(0, Math.floor((ly + widthM / 2) / resolutionM)))
  return { ix, iy }
}

function classifyTerrainCell(
  cell: number,
  x: number,
  y: number,
  width: number,
  height: number,
): keyof typeof TERRAIN_PIXEL {
  const centerX = Math.floor(width / 2)
  const robotRows = Math.max(2, Math.floor(height * 0.012))
  const halfBand = Math.max(1, Math.floor(width * 0.01))
  if (y < robotRows && Math.abs(x - centerX) <= halfBand) return 'robot'
  if (cell >= 90) return 'bad'
  if (cell > 0) return 'warn'
  if (cell < 0) return 'unknown'
  return 'ok'
}

function fallbackTerrainCategory(x: number, y: number, w: number, h: number): keyof typeof TERRAIN_PIXEL {
  const x12 = Math.min(11, Math.floor((x * 12) / w))
  const y12 = Math.min(11, Math.floor((y * 12) / h))
  if ((x12 === 5 || x12 === 6) && y12 > 7) return 'robot'
  if ((x12 > 7 && y12 < 4) || (x12 === 2 && y12 === 5) || (x12 === 3 && y12 === 5)) return 'bad'
  const cornerR = Math.max(3, Math.round(Math.min(w, h) * 0.04))
  if ((x < cornerR && y < cornerR) || (x >= w - cornerR && y >= h - cornerR)) return 'unknown'
  if (x12 === 7 && y12 === 6) return 'warn'
  return 'ok'
}

function TerrainGrid({
  grid,
  zones,
  zoneMarking,
  pickArmed,
  pickEnabled,
  onPick,
}: {
  grid?: TerrainGridSnapshot
  zones?: FieldZone[]
  zoneMarking?: ZoneMarkingSnapshot
  pickArmed?: boolean
  pickEnabled?: boolean
  onPick?: (p: { x: number; y: number }) => void
}) {
  const hasLiveCells = Boolean(grid && grid.width > 0 && grid.height > 0 && grid.cells.length === grid.width * grid.height)
  const width = hasLiveCells ? grid!.width : 30
  const height = hasLiveCells ? grid!.height : 30
  const resolutionM = hasLiveCells ? grid!.resolution : 0.1
  const categories = useMemo(() => {
    if (hasLiveCells) {
      return grid!.cells.map((cell, index) => {
        const x = index % width
        const y = Math.floor(index / width)
        return classifyTerrainCell(cell, x, y, width, height)
      })
    }
    return Array.from({ length: width * height }, (_, index) => {
      const x = index % width
      const y = Math.floor(index / width)
      return fallbackTerrainCategory(x, y, width, height)
    })
  }, [grid, hasLiveCells, height, width])

  const cellBackingPx = useMemo(() => {
    const maxDim = Math.max(width, height)
    // Backing store ~960px on the long edge so fine grids stay legible; 2–8 px per cell.
    return Math.min(8, Math.max(2, Math.floor(960 / maxDim)))
  }, [width, height])

  const maxGridDim = Math.max(width, height)
  const panelMaxClass =
    maxGridDim <= 48
      ? 'max-w-[200px] sm:max-w-[220px]'
      : maxGridDim <= 120
        ? 'max-w-[min(92vw,440px)]'
        : 'max-w-[min(96vw,min(900px,100vw-1.5rem))]'

  const canvasRef = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    const px = cellBackingPx
    canvas.width = width * px
    canvas.height = height * px
    ctx.imageSmoothingEnabled = false
    for (let i = 0; i < categories.length; i++) {
      const x = i % width
      const y = Math.floor(i / width)
      const cat = categories[i]!
      ctx.fillStyle = TERRAIN_PIXEL[cat]
      ctx.fillRect(x * px, y * px, px, px)
    }

    const zm = zoneMarking
    if (zm && zones && hasLiveCells) {
      const res = resolutionM
      const yawEff = zm.yawDeg ?? 0
      const rpx = Math.max(2, Math.min(6, Math.ceil(px / 2)))
      for (const z of zones) {
        if (z.status !== 'operator_marked') continue
        const zx = z.odom_x
        const zy = z.odom_y
        if (typeof zx !== 'number' || typeof zy !== 'number' || !Number.isFinite(zx) || !Number.isFinite(zy)) continue
        const cell = odomToTerrainCell(zm.x, zm.y, yawEff, zx, zy, width, height, res)
        if (!cell) continue
        const { ix, iy } = cell
        ctx.fillStyle = ZONE_MARKER_RGBA[z.id]
        ctx.fillRect(ix * px - rpx, iy * px - rpx, px + 2 * rpx, px + 2 * rpx)
        ctx.strokeStyle = 'rgba(0,0,0,0.55)'
        ctx.lineWidth = Math.max(1, px > 3 ? 2 : 1)
        ctx.strokeRect(ix * px - rpx + 0.5, iy * px - rpx + 0.5, px + 2 * rpx - 1, px + 2 * rpx - 1)
      }
    }
  }, [categories, cellBackingPx, height, width, zones, zoneMarking, hasLiveCells, resolutionM])

  function clickCanvas(e: React.MouseEvent<HTMLCanvasElement>) {
    if (!pickArmed || !pickEnabled || !onPick) return
    const canvas = e.currentTarget
    const rect = canvas.getBoundingClientRect()
    const fx = (e.clientX - rect.left) / Math.max(rect.width, 1e-6)
    const fy = (e.clientY - rect.top) / Math.max(rect.height, 1e-6)
    const ix = Math.min(width - 1, Math.max(0, Math.floor(fx * width)))
    const iy = Math.min(height - 1, Math.max(0, Math.floor(fy * height)))
    const widthM = width * resolutionM
    const lx = (iy + 0.5) * resolutionM
    const ly = -widthM / 2 + (ix + 0.5) * resolutionM
    onPick({ x: lx, y: ly })
  }

  const picking = Boolean(pickArmed && pickEnabled && onPick)

  return (
    <div className={clsx('mx-auto w-full space-y-2 md:mx-0', panelMaxClass)}>
      <div
        className={clsx(
          'rounded-md border bg-slate-950 p-1.5 sm:p-2',
          picking ? 'cursor-crosshair border-amber-500/60 ring-1 ring-amber-500/40' : 'border-slate-800',
        )}
      >
        <canvas
          ref={canvasRef}
          role="img"
          aria-label="Local terrain traversability grid"
          className={clsx('aspect-square h-auto w-full [image-rendering:pixelated]', picking && 'cursor-crosshair')}
          onClick={clickCanvas}
        />
      </div>
      <div className="flex flex-wrap items-center justify-between gap-2 text-[11px] text-slate-500">
        <span>{hasLiveCells ? `${width}×${height} @ ${grid!.resolution.toFixed(2)} m/cell` : `mock ${width}×${height} @ 0.10 m/cell`}</span>
        <span>{hasLiveCells ? `${grid!.status}, age ${grid!.ageMs ?? '--'} ms` : 'waiting for /autonomy/local_terrain_grid'}</span>
      </div>
      <p className="text-[10px] leading-snug text-slate-500">
        Terrain: <span className="text-slate-400">gray</span> = unknown, <span className="text-emerald-400/90">green</span> = traversable,{' '}
        <span className="text-amber-300/90">yellow</span> = uneven / drop-off risk (slow down), <span className="text-red-400/90">red</span> = obstacle; white strip =
        robot forward. Markers:{' '}
        <span className="text-emerald-300/90">start</span>, <span className="text-amber-300/90">dig</span>, <span className="text-orange-300/90">dump</span>,{' '}
        <span className="text-red-400/90">no-go</span>.
      </p>
    </div>
  )
}

function MiniTrend({
  title,
  values,
  suffix = '',
}: {
  title: string
  values: number[]
  suffix?: string
}) {
  const max = Math.max(...values, 1)
  return (
    <div className="rounded-md border border-slate-800 bg-slate-900 p-3">
      <div className="flex items-center justify-between text-xs text-slate-400">
        <span>{title}</span>
        <span>{values.at(-1) ?? 0}{suffix}</span>
      </div>
      <div className="mt-3 flex h-20 items-end gap-1">
        {values.map((value, index) => (
          <div
            key={`${title}-${index}`}
            className="flex-1 rounded-t bg-cyan-400/70"
            style={{ height: `${Math.max(8, (value / max) * 100)}%` }}
          />
        ))}
      </div>
    </div>
  )
}

function IrRadar({ left, right }: { left: number | null; right: number | null }) {
  const leftPct = left === null ? 0 : Math.min(100, (left / 1024) * 100)
  const rightPct = right === null ? 0 : Math.min(100, (right / 1024) * 100)
  return (
    <div className="rounded-md border border-slate-800 bg-slate-900 p-3">
      <div className="text-xs uppercase tracking-wide text-slate-500">IR proximity radar</div>
      <div className="mt-3 grid grid-cols-2 gap-3">
        <div>
          <div className="mb-1 flex justify-between text-xs text-slate-400">
            <span>Left</span>
            <span>{left ?? '--'}</span>
          </div>
          <div className="h-3 overflow-hidden rounded bg-slate-800">
            <div className="h-full bg-amber-300" style={{ width: `${leftPct}%` }} />
          </div>
        </div>
        <div>
          <div className="mb-1 flex justify-between text-xs text-slate-400">
            <span>Right</span>
            <span>{right ?? '--'}</span>
          </div>
          <div className="h-3 overflow-hidden rounded bg-slate-800">
            <div className="h-full bg-amber-300" style={{ width: `${rightPct}%` }} />
          </div>
        </div>
      </div>
      <div className="mt-3 flex h-28 items-end justify-center gap-16 rounded border border-slate-800 bg-slate-950 p-3">
        <div className="h-20 w-8 origin-bottom -rotate-45 rounded-t-full bg-amber-400/60" style={{ transform: `rotate(-45deg) scaleY(${Math.max(0.18, leftPct / 100)})` }} />
        <div className="h-20 w-8 origin-bottom rotate-45 rounded-t-full bg-amber-400/60" style={{ transform: `rotate(45deg) scaleY(${Math.max(0.18, rightPct / 100)})` }} />
      </div>
    </div>
  )
}

function MissionOverview({ snapshot }: { snapshot: MissionControlSnapshot }) {
  const { mission } = snapshot
  return (
    <Panel title="Mission State" icon={<Activity className="h-4 w-4 text-cyan-300" />}>
      <div className="grid gap-3 lg:grid-cols-[1.1fr_1fr]">
        <div>
          <div className="flex flex-wrap items-center gap-3">
            <div className="text-2xl font-semibold text-white sm:text-3xl">{mission.state}</div>
            <span className="rounded border border-amber-500/40 bg-amber-500/10 px-2 py-1 text-xs text-amber-100">{mission.mode}</span>
          </div>
          <p className="mt-2 text-sm text-slate-300">{mission.stopReason}</p>
          <div className="mt-4 grid gap-2 text-sm sm:grid-cols-2">
            <div className="rounded border border-slate-800 bg-slate-900 p-3">
              <div className="text-xs text-slate-500">Last decision</div>
              <div className="mt-1 text-slate-100">{mission.lastDecision}</div>
            </div>
            <div className="rounded border border-slate-800 bg-slate-900 p-3">
              <div className="text-xs text-slate-500">Next transition</div>
              <div className="mt-1 text-slate-100">{mission.nextTransition}</div>
            </div>
          </div>
        </div>
        <div className="grid grid-cols-3 gap-2">
          <MetricCard metric={{ label: 'Confidence', value: `${mission.confidence}%`, detail: 'mission', status: mission.confidence > 85 ? 'ok' : mission.confidence > 0 ? 'warn' : 'bad' }} />
          <MetricCard metric={{ label: 'Payload', value: mission.payload, detail: 'bucket', status: 'idle' }} />
          <MetricCard metric={{ label: 'Cycle', value: String(mission.cycle), detail: 'dig/dump', status: 'idle' }} />
        </div>
      </div>
    </Panel>
  )
}

function FallbackDiagnostics({ snapshot }: { snapshot: MissionControlSnapshot }) {
  return (
    <Panel title="Connection And Data Fallbacks" icon={<WifiOff className="h-4 w-4 text-amber-300" />}>
      <div className="grid gap-3 lg:grid-cols-3">
        <EmptyState title="Robot bridge fallback" detail="If the bridge disconnects, freeze telemetry, disable motion buttons, and keep ESTOP visible." status={snapshot.mission.connected ? 'ok' : 'bad'} />
        <EmptyState title="No camera stream fallback" detail="Show a clear no-stream panel with topic name, stream age, and reconnect status." status={snapshot.cameras.some((camera) => camera.status === 'missing') ? 'bad' : 'ok'} />
        <EmptyState title="ROS topic stale fallback" detail="Any stale safety-critical topic should show age/rate and block autonomous start." status={snapshot.topics.some((topic) => topic.safetyCritical && topic.status !== 'live') ? 'warn' : 'ok'} />
      </div>
      <div className="mt-4 overflow-x-auto rounded-md border border-slate-800">
        <table className="w-full min-w-[760px] text-left text-sm">
          <thead className="bg-slate-900 text-xs uppercase text-slate-500">
            <tr>
              <th className="px-3 py-2">Topic</th>
              <th className="px-3 py-2">Status</th>
              <th className="px-3 py-2">Age</th>
              <th className="px-3 py-2">Rate</th>
              <th className="px-3 py-2">Note</th>
            </tr>
          </thead>
          <tbody>
            {snapshot.topics.map((row) => {
              const status = streamToStatus(row.status)
              return (
                <tr key={row.topic} className="border-t border-slate-800 bg-slate-950">
                  <td className="px-3 py-2 font-mono text-xs text-slate-100">{row.topic}</td>
                  <td className="px-3 py-2">
                    <span className={clsx('inline-flex rounded border px-2 py-1 text-xs', statusClass(status))}>{row.status}</span>
                  </td>
                  <td className="px-3 py-2 text-slate-400">{row.age}</td>
                  <td className="px-3 py-2 text-slate-400">{row.rate}</td>
                  <td className="px-3 py-2 text-slate-400">{row.note}</td>
                </tr>
              )
            })}
          </tbody>
        </table>
      </div>
    </Panel>
  )
}

function fmtNum(value: number | null | undefined, digits = 0): string {
  if (value == null || !Number.isFinite(value)) return '—'
  return value.toFixed(digits)
}

function TeleopTelemetry({ snapshot }: { snapshot: MissionControlSnapshot }) {
  const a = snapshot.actuators
  const s = snapshot.sensors
  const bucketPos = a?.bucketPos ?? snapshot.mission.digSequence?.bucketPosCommanded
  return (
    <div className="mt-4 grid gap-2 text-[11px] text-slate-400 sm:grid-cols-2 xl:grid-cols-3">
      <div className="rounded-md border border-slate-800 bg-slate-950/70 p-2">
        <div className="text-[10px] font-semibold uppercase text-slate-500">Drive</div>
        <div className="mt-1 font-mono text-slate-200">
          lin={fmtNum(a?.driveLinear, 2)} ang={fmtNum(a?.driveAngular, 2)}
        </div>
        <div className="mt-1 text-slate-500">cmd/velocity echo</div>
      </div>
      <div className="rounded-md border border-slate-800 bg-slate-950/70 p-2">
        <div className="text-[10px] font-semibold uppercase text-slate-500">Servos</div>
        <div className="mt-1 font-mono text-slate-200">
          cam={fmtNum(a?.cameraHeight)}% pan={fmtNum(a?.panAngle)}
        </div>
        <div className="mt-1 text-slate-500">/cmd/camera_height · /cmd/pan</div>
      </div>
      <div className="rounded-md border border-slate-800 bg-slate-950/70 p-2">
        <div className="text-[10px] font-semibold uppercase text-slate-500">Mining</div>
        <div className="mt-1 font-mono text-slate-200">
          bucket={fmtNum(bucketPos)}/{fmtNum(a?.bucketPosMax ?? 40)}% chain={fmtNum(a?.bucketVel)} conv={a?.conveyor ? 'ON' : 'OFF'}
        </div>
        <div className="mt-1 text-slate-500">bucket pos / chain / conveyor</div>
      </div>
      <div className="rounded-md border border-slate-800 bg-slate-950/70 p-2">
        <div className="text-[10px] font-semibold uppercase text-slate-500">IR</div>
        <div className="mt-1 font-mono text-slate-200">
          left={fmtNum(s.irLeft)} right={fmtNum(s.irRight)}
        </div>
        <div className="mt-1 text-slate-500">/sensor/ir/left · /sensor/ir/right</div>
      </div>
      <div className="rounded-md border border-slate-800 bg-slate-950/70 p-2">
        <div className="text-[10px] font-semibold uppercase text-slate-500">Encoders</div>
        <div className="mt-1 font-mono text-slate-200">
          left={fmtNum(s.encoderLeft)} right={fmtNum(s.encoderRight)}
        </div>
        <div className="mt-1 text-slate-500">/sensor/encoder/*</div>
      </div>
      <div className="rounded-md border border-slate-800 bg-slate-950/70 p-2">
        <div className="text-[10px] font-semibold uppercase text-slate-500">Encoder Debug</div>
        <div className="mt-1 font-mono text-slate-200">
          pin=({fmtNum(s.encoderPinRight)}/{fmtNum(s.encoderPinLeft)}) dec=({fmtNum(s.encoderDecRight)}/{fmtNum(s.encoderDecLeft)})
        </div>
        <div className="mt-1 font-mono text-slate-500">
          bad=({fmtNum(s.encoderBadRight)}/{fmtNum(s.encoderBadLeft)})
        </div>
      </div>
    </div>
  )
}

function App() {
  const [scenario, setScenarioState] = useState<DemoScenario>(initialDemoScenario)
  const [commandStatus, setCommandStatus] = useState('No command sent in this session.')
  const [bagName, setBagName] = useState('mission_data')
  const [mapName, setMapName] = useState('lunar_field')
  const [pid, setPid] = useState({ kp: 1.0, ki: 0.0, kd: 0.1 })
  const [zoneArm, setZoneArm] = useState<FieldZone['id'] | null>(null)
  const [digCycles, setDigCycles] = useState(8)

  function setScenario(next: DemoScenario) {
    setZoneArm(null)
    setScenarioState(next)
  }

  const { mode, setMode: setBridgeModeInternal, snapshot, sendCommand, liveAvailable, liveStatus, liveUrl } = useMissionBridge(scenario)

  const cameraWsBase = useCameraWsBaseUrl()
  const sensorOverlay = useSensorWebSocket(cameraWsBase, snapshot)
  const displaySnapshot = useMemo(() => {
    if (!sensorOverlay) return snapshot
    const overlaySensors = Object.fromEntries(
      Object.entries(sensorOverlay.sensors).filter(([, value]) => value != null),
    )
    const overlayActuators = Object.fromEntries(
      Object.entries(sensorOverlay.actuators).filter(([, value]) => value != null),
    )
    return {
      ...snapshot,
      trends: sensorOverlay.trends ?? snapshot.trends,
      sensors: { ...snapshot.sensors, ...overlaySensors },
      actuators: { ...snapshot.actuators, ...overlayActuators },
      metrics: sensorOverlay.metrics,
      logs: sensorOverlay.logs,
    }
  }, [snapshot, sensorOverlay])

  function setMode(next: 'mock' | 'live') {
    setZoneArm(null)
    setBridgeModeInternal(next)
  }

  const motionDisabled = !displaySnapshot.mission.connected
  const drive = (command: 'forward' | 'reverse' | 'left' | 'right', speedLimit = 30) =>
    commandButtonState(mode, displaySnapshot.mission.connected, { type: 'drive', command, speedLimit })
  const driveStop = commandButtonState(mode, displaySnapshot.mission.connected, {
    type: 'drive',
    command: 'stop',
    speedLimit: 0,
  })
  const actuator = (
    target: 'pan' | 'camera_height' | 'bucket_pos' | 'bucket_vel' | 'conveyor',
    action: 'increment' | 'decrement' | 'stop' | 'toggle',
  ) => commandButtonState(mode, displaySnapshot.mission.connected, { type: 'actuator', target, action })

  const odomLive =
    displaySnapshot.zoneMarking?.odomStatus === 'live' ||
    displaySnapshot.topics.some((t) => t.topic === '/odom' && t.status === 'live')
  /** Live + connected bridge + stale/missing odom: block (real pose unknown). Otherwise allow UI / mock playtest. */
  const zoneMarkingBlockedByOdom = mode === 'live' && liveStatus.connected && !odomLive
  const canZonePick = displaySnapshot.mission.connected && !zoneMarkingBlockedByOdom

  async function runCommand(command: Parameters<typeof sendCommand>[0]) {
    const result = await sendCommand(command)
    setCommandStatus(`${result.accepted ? 'Accepted' : 'Rejected'}: ${result.message}`)
  }

  return (
    <div className="min-h-screen bg-[#080b10] text-slate-200">
      <SafetyBar
        snapshot={displaySnapshot}
        scenario={scenario}
        setScenario={setScenario}
        bridgeMode={mode}
        setBridgeMode={setMode}
        liveAvailable={liveAvailable}
        runCommand={runCommand}
      />
      <main className="mx-auto flex max-w-[1800px] flex-col gap-4 px-3 py-4 sm:px-4">
        {mode === 'mock' ? (
          <EmptyState
            title="Mock bridge"
            detail="UI rehearsal only — commands are not sent to ROS. Select Live bridge when lunar mission-bridge is running on port 8770."
            status="warn"
          />
        ) : (
          <EmptyState
            title={liveStatus.connected ? 'Live bridge connected' : liveStatus.connecting ? 'Live bridge connecting' : 'Live bridge disconnected'}
            detail={liveUrl ? `${liveUrl}${liveStatus.lastError ? ` | ${liveStatus.lastError}` : ''}` : 'Set VITE_MISSION_WS_URL to enable live mode.'}
            status={liveStatus.connected ? 'ok' : liveStatus.connecting ? 'warn' : 'bad'}
          />
        )}
        <section className="grid gap-3 sm:grid-cols-2 xl:grid-cols-6">
          {displaySnapshot.metrics.map((metric) => (
            <MetricCard key={metric.label} metric={metric} />
          ))}
        </section>

        <MissionOverview snapshot={displaySnapshot} />
        <EmptyState title="Latest command result" detail={commandStatus} status={commandStatus.startsWith('Rejected') ? 'bad' : commandStatus.startsWith('Accepted') ? 'ok' : 'idle'} />
        <FallbackDiagnostics snapshot={displaySnapshot} />

        <section className="grid gap-4 xl:grid-cols-[1.4fr_1fr]">
          <Panel
            title="Live Cameras"
            icon={<Video className="h-4 w-4 text-cyan-300" />}
            action={
              <span className="text-xs text-slate-500">
                {cameraWsBase
                  ? `${cameraWsBase} · /camera/ws/*${sensorOverlay && !sensorOverlay.connected ? ' · /sensor/ws reconnecting' : ''}`
                  : 'Resolving camera bridge URL (same host, port from VITE_CAMERA_WS_PORT or 8767)…'}
              </span>
            }
          >
            <div className="grid gap-3 lg:grid-cols-2">
              {displaySnapshot.cameras.map((camera) => (
                <CameraFeed key={camera.id} camera={camera} wsBase={cameraWsBase} />
              ))}
            </div>
          </Panel>

          <Panel title="Local Terrain And Navigation" icon={<MapPinned className="h-4 w-4 text-emerald-300" />}>
            <div className="grid gap-4 md:grid-cols-[minmax(0,220px)_1fr] md:items-start">
              <div className="space-y-1.5">
                {zoneArm ? <p className="text-center text-xs text-amber-200">Click map</p> : null}
                <TerrainGrid
                  grid={displaySnapshot.terrainGrid}
                  zones={displaySnapshot.zones}
                  zoneMarking={displaySnapshot.zoneMarking}
                  pickArmed={zoneArm !== null}
                  pickEnabled={canZonePick}
                  onPick={(p) => {
                    if (!zoneArm) return
                    void (async () => {
                      const result = await sendCommand({
                        type: 'mark_zone',
                        zone: zoneArm,
                        pick: { frameId: 'base_link', x: p.x, y: p.y },
                      })
                      setCommandStatus(`${result.accepted ? 'Accepted' : 'Rejected'}: ${result.message}`)
                      setZoneArm(null)
                    })()
                  }}
                />
              </div>
              <div className="min-w-0 space-y-3">
                <div className="grid grid-cols-2 gap-2">
                  <MetricCard metric={{ label: 'Obstacles', value: displaySnapshot.terrainGrid ? String(displaySnapshot.terrainGrid.obstacleCells) : '--', detail: displaySnapshot.terrainGrid?.note ?? 'terrain grid', status: displaySnapshot.terrainGrid?.status === 'live' ? 'ok' : displaySnapshot.terrainGrid ? 'warn' : 'bad' }} />
                  <MetricCard metric={{ label: 'Rough / drop risk', value: displaySnapshot.terrainGrid ? String(displaySnapshot.terrainGrid.cautionCells) : '--', detail: 'yellow cells (value 60)', status: displaySnapshot.terrainGrid && displaySnapshot.terrainGrid.cautionCells > 0 ? 'warn' : displaySnapshot.terrainGrid ? 'ok' : 'bad' }} />
                  <MetricCard metric={{ label: 'Unknown cells', value: displaySnapshot.terrainGrid ? String(displaySnapshot.terrainGrid.unknownCells) : '--', detail: displaySnapshot.terrainGrid?.frameId ?? 'base_link', status: displaySnapshot.terrainGrid && displaySnapshot.terrainGrid.unknownCells > 0 ? 'warn' : displaySnapshot.terrainGrid ? 'ok' : 'bad' }} />
                  <MetricCard metric={steeringMetric(displaySnapshot)} />
                </div>
                <div className="flex flex-wrap gap-2">
                  <Button disabled title="Terrain map editing is not wired on the dashboard yet.">Clear Map</Button>
                  <Button disabled title="Terrain freeze is not wired on the dashboard yet.">Freeze</Button>
                  <Button disabled title="Hazard overlay toggle is not wired on the dashboard yet.">Toggle Hazards</Button>
                </div>
              </div>
            </div>
          </Panel>
        </section>

        <section className="grid gap-4 xl:grid-cols-[1fr_1.2fr_1fr]">
          <Panel title="Zone marking" icon={<Flag className="h-4 w-4 text-orange-300" />}>
            <p className="mb-3 text-xs leading-relaxed text-slate-400">
              <span className="font-medium text-slate-300">Odometry (&quot;odom&quot;)</span> is the robot&apos;s estimated position and heading
              (usually from wheel encoders plus IMU), published on the ROS topic{' '}
              <span className="font-mono text-slate-300">/odom</span>. Zone marks need that pose so the dig/dump points land in the right place on the
              field. On a <span className="text-slate-300">real robot</span>, marking stays disabled until the live bridge reports{' '}
              <span className="font-mono text-slate-300">/odom</span> as live. Use{' '}
              <span className="text-slate-300">Mock bridge</span> in the header to rehearse Prepare mark + map clicks with fake telemetry (pick any
              scenario except Offline).
            </p>
            <ZoneMarkingPanel snapshot={displaySnapshot} armedZone={zoneArm} onArm={setZoneArm} canArm={canZonePick} />
            {zoneMarkingBlockedByOdom ? (
              <p className="mt-2 text-xs text-amber-200/90">
                Live bridge is connected but <span className="font-mono">/odom</span> is not live yet — fix localization, or switch to Mock bridge to
                practice the UI.
              </p>
            ) : null}
          </Panel>

          <Panel title="Manual Teleop" icon={<Gauge className="h-4 w-4 text-cyan-300" />}>
            <div className="grid gap-4 lg:grid-cols-2">
              <div>
                <div className="grid grid-cols-3 gap-2">
                  <div />
                  <Button disabled={drive('forward').disabled} title={drive('forward').title} onClick={() => void runCommand({ type: 'drive', command: 'forward', speedLimit: 30 })}><ArrowUp className="h-4 w-4" /> Fwd</Button>
                  <div />
                  <Button disabled={drive('left').disabled} title={drive('left').title} onClick={() => void runCommand({ type: 'drive', command: 'left', speedLimit: 30 })}><ArrowLeft className="h-4 w-4" /> Left</Button>
                  <Button intent="danger" disabled={driveStop.disabled} title={driveStop.title} onClick={() => void runCommand({ type: 'drive', command: 'stop', speedLimit: 0 })}><Square className="h-4 w-4" /> Stop</Button>
                  <Button disabled={drive('right').disabled} title={drive('right').title} onClick={() => void runCommand({ type: 'drive', command: 'right', speedLimit: 30 })}>Right <ArrowRight className="h-4 w-4" /></Button>
                  <div />
                  <Button disabled={drive('reverse').disabled} title={drive('reverse').title} onClick={() => void runCommand({ type: 'drive', command: 'reverse', speedLimit: 30 })}><ArrowDown className="h-4 w-4" /> Rev</Button>
                  <div />
                </div>
                <div className="mt-3">
                  <label className="text-xs text-slate-500">Speed limit</label>
                  <input disabled={motionDisabled} className="mt-1 w-full accent-cyan-400 disabled:opacity-40" type="range" min="0" max="100" defaultValue="30" />
                </div>
              </div>
              <div className="grid grid-cols-2 gap-2 text-sm">
                <Button disabled={actuator('pan', 'decrement').disabled} title={actuator('pan', 'decrement').title} onClick={() => void runCommand({ type: 'actuator', target: 'pan', action: 'decrement' })}><Camera className="h-4 w-4" /> Pan Left</Button>
                <Button disabled={actuator('pan', 'increment').disabled} title={actuator('pan', 'increment').title} onClick={() => void runCommand({ type: 'actuator', target: 'pan', action: 'increment' })}>Pan Right</Button>
                <Button disabled={actuator('camera_height', 'increment').disabled} title={actuator('camera_height', 'increment').title} onClick={() => void runCommand({ type: 'actuator', target: 'camera_height', action: 'increment' })}>Cam Up</Button>
                <Button disabled={actuator('camera_height', 'decrement').disabled} title={actuator('camera_height', 'decrement').title} onClick={() => void runCommand({ type: 'actuator', target: 'camera_height', action: 'decrement' })}>Cam Down</Button>
                <Button disabled={actuator('bucket_pos', 'increment').disabled} title={actuator('bucket_pos', 'increment').title} onClick={() => void runCommand({ type: 'actuator', target: 'bucket_pos', action: 'increment' })}>Bucket Up</Button>
                <Button disabled={actuator('bucket_pos', 'decrement').disabled} title={actuator('bucket_pos', 'decrement').title} onClick={() => void runCommand({ type: 'actuator', target: 'bucket_pos', action: 'decrement' })}>Bucket Down</Button>
                <Button disabled={actuator('bucket_vel', 'increment').disabled} title={actuator('bucket_vel', 'increment').title} onClick={() => void runCommand({ type: 'actuator', target: 'bucket_vel', action: 'increment' })}>Chain Fwd</Button>
                <Button disabled={actuator('bucket_vel', 'decrement').disabled} title={actuator('bucket_vel', 'decrement').title} onClick={() => void runCommand({ type: 'actuator', target: 'bucket_vel', action: 'decrement' })}>Chain Rev</Button>
                <Button disabled={actuator('conveyor', 'toggle').disabled} title={actuator('conveyor', 'toggle').title} className="col-span-2" onClick={() => void runCommand({ type: 'actuator', target: 'conveyor', action: 'toggle' })}>Conveyor Toggle</Button>
              </div>
            </div>
            <TeleopTelemetry snapshot={displaySnapshot} />
          </Panel>

          <Panel title="Mining Cycle" icon={<Shovel className="h-4 w-4 text-amber-300" />}>
            <div className="grid gap-2">
              <MetricCard
                metric={{
                  label: 'Dig cycles',
                  value: String(displaySnapshot.mission.digSequence?.maxCyclesLe ?? digCycles),
                  detail: displaySnapshot.mission.digSequence?.phase ?? 'requested',
                  status: displaySnapshot.mission.digSequence ? 'ok' : 'idle',
                }}
              />
              <label className="text-xs text-slate-500">Dig cycles</label>
              <input
                className="w-full rounded border border-slate-700 bg-slate-900 px-2 py-1 text-sm disabled:opacity-40"
                type="number"
                min="1"
                max="100"
                step="1"
                value={digCycles}
                disabled={motionDisabled}
                onChange={(event) => {
                  const next = Math.trunc(Number(event.target.value))
                  setDigCycles(Number.isFinite(next) ? Math.max(1, Math.min(100, next)) : 8)
                }}
              />
              <div className="grid grid-cols-2 gap-2">
                <Button disabled={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'dig_start', cycles: digCycles }).disabled} title={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'dig_start', cycles: digCycles }).title} intent="safe" onClick={() => void runCommand({ type: 'payload', command: 'dig_start', cycles: digCycles })}>Start Dig</Button>
                <Button disabled={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'dig_stop' }).disabled} title={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'dig_stop' }).title} onClick={() => void runCommand({ type: 'payload', command: 'dig_stop' })}>Stop Dig</Button>
                <Button disabled={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'dump_start' }).disabled} title={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'dump_start' }).title} intent="safe" onClick={() => void runCommand({ type: 'payload', command: 'dump_start' })}>Start Dump</Button>
                <Button disabled={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'dump_stop' }).disabled} title={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'dump_stop' }).title} onClick={() => void runCommand({ type: 'payload', command: 'dump_stop' })}>Stop Dump</Button>
                <Button disabled={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'stow' }).disabled} title={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'stow' }).title} onClick={() => void runCommand({ type: 'payload', command: 'stow' })}><Home className="h-4 w-4" /> Stow</Button>
                <Button disabled={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'abort' }).disabled} title={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'payload', command: 'abort' }).title} intent="danger" onClick={() => void runCommand({ type: 'payload', command: 'abort' })}>Abort Payload</Button>
              </div>
              <label className="text-xs text-slate-500">Dig duration</label>
              <input disabled={motionDisabled} className="w-full rounded border border-slate-700 bg-slate-900 px-2 py-1 text-sm disabled:opacity-40" defaultValue="8 s" />
            </div>
          </Panel>
        </section>

        <section className="grid gap-4 xl:grid-cols-[1fr_1fr]">
          <Panel title="Analytics And Pulse" icon={<BarChart3 className="h-4 w-4 text-cyan-300" />}>
            <div className="grid gap-3 lg:grid-cols-3">
              <MiniTrend title="CPU load" values={displaySnapshot.trends.map((point) => point.cpu)} suffix="%" />
              <MiniTrend title="CPU temp" values={displaySnapshot.trends.map((point) => point.temp)} suffix=" C" />
              <MiniTrend title="Battery" values={displaySnapshot.trends.map((point) => point.battery)} suffix=" V" />
            </div>
            <div className="mt-3 grid gap-3 lg:grid-cols-[1.2fr_1fr]">
              <div className="rounded-md border border-slate-800 bg-slate-900 p-3">
                <div className="text-xs uppercase tracking-wide text-slate-500">Odometry vs command velocity</div>
                <div className="mt-3 grid grid-cols-12 items-end gap-1">
                  {displaySnapshot.trends.map((point) => (
                    <div key={point.label} className="flex h-24 flex-col justify-end gap-1">
                      <div className="rounded-t bg-purple-400/70" style={{ height: `${Math.max(6, point.odomVelocity * 300)}px` }} title={`odom ${point.odomVelocity}`} />
                      <div className="rounded-t bg-cyan-300/80" style={{ height: `${Math.max(6, point.commandVelocity * 300)}px` }} title={`cmd ${point.commandVelocity}`} />
                    </div>
                  ))}
                </div>
                <div className="mt-2 flex gap-4 text-xs text-slate-400">
                  <span className="inline-flex items-center gap-1"><span className="h-2 w-2 rounded bg-purple-400" /> odom</span>
                  <span className="inline-flex items-center gap-1"><span className="h-2 w-2 rounded bg-cyan-300" /> command</span>
                </div>
              </div>
              <IrRadar left={displaySnapshot.sensors.irLeft} right={displaySnapshot.sensors.irRight} />
            </div>
          </Panel>

          <Panel title="Sensor And Hardware Health" icon={<Cpu className="h-4 w-4 text-cyan-300" />}>
            <div className="overflow-x-auto rounded-md border border-slate-800">
              <table className="w-full min-w-[700px] text-left text-sm">
                <thead className="bg-slate-900 text-xs uppercase text-slate-500">
                  <tr>
                    <th className="px-3 py-2">Category</th>
                    <th className="px-3 py-2">Name</th>
                    <th className="px-3 py-2">Status</th>
                    <th className="px-3 py-2">Detail</th>
                  </tr>
                </thead>
                <tbody>
                  {displaySnapshot.hardware.map((row) => (
                    <tr key={row.name} className="border-t border-slate-800 bg-slate-950">
                      <td className="px-3 py-2 text-slate-400">{row.category}</td>
                      <td className="px-3 py-2 text-slate-100">{row.name}</td>
                      <td className="px-3 py-2"><span className={clsx('rounded border px-2 py-1 text-xs', statusClass(row.severity))}>{row.status}</span></td>
                      <td className="px-3 py-2 text-slate-400">{row.detail}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </Panel>

          <Panel title="Data, Logs, And Diagnostics" icon={<Database className="h-4 w-4 text-emerald-300" />}>
            <div className="grid gap-3 lg:grid-cols-2">
              <div className="space-y-2">
                <div className="grid grid-cols-2 gap-2">
                  <label className="text-xs text-slate-500">
                    Bag name
                    <input
                      value={bagName}
                      onChange={(event) => setBagName(event.target.value)}
                      className="mt-1 w-full rounded border border-slate-700 bg-slate-900 px-2 py-1 text-sm text-slate-100"
                    />
                  </label>
                  <label className="text-xs text-slate-500">
                    Map name
                    <input
                      value={mapName}
                      onChange={(event) => setMapName(event.target.value)}
                      className="mt-1 w-full rounded border border-slate-700 bg-slate-900 px-2 py-1 text-sm text-slate-100"
                    />
                  </label>
                </div>
                <div className="grid grid-cols-2 gap-2">
                  <Button disabled={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'recording', enabled: true, name: bagName }).disabled} title={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'recording', enabled: true, name: bagName }).title} intent="danger" onClick={() => void runCommand({ type: 'recording', enabled: true, name: bagName })}>Start Bag</Button>
                  <Button disabled={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'recording', enabled: false }).disabled} title={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'recording', enabled: false }).title} onClick={() => void runCommand({ type: 'recording', enabled: false })}>Stop Bag</Button>
                  <Button disabled={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'save_map', name: mapName }).disabled} title={commandButtonState(mode, displaySnapshot.mission.connected, { type: 'save_map', name: mapName }).title} onClick={() => void runCommand({ type: 'save_map', name: mapName })}><Save className="h-4 w-4" /> Save Map</Button>
                  <Button disabled title="Event markers are not wired on the dashboard yet.">Event Marker</Button>
                </div>
                <div className="grid grid-cols-2 gap-2">
                  <MetricCard metric={{ label: 'CPU', value: displaySnapshot.mission.connected ? '29%' : '--', detail: 'demo', status: displaySnapshot.mission.connected ? 'ok' : 'bad' }} />
                  <MetricCard metric={{ label: 'Temp', value: displaySnapshot.mission.connected ? '54 C' : '--', detail: 'Jetson', status: displaySnapshot.mission.connected ? 'ok' : 'bad' }} />
                  <MetricCard metric={{ label: 'WiFi', value: displaySnapshot.mission.connected ? '18 ms' : '--', detail: 'ping', status: displaySnapshot.mission.connected ? 'ok' : 'bad' }} />
                  <MetricCard metric={{ label: 'Disk', value: displaySnapshot.mission.connected ? '72 GB' : '--', detail: 'bags', status: displaySnapshot.mission.connected ? 'ok' : 'bad' }} />
                </div>
              </div>
              <div className="rounded-md border border-slate-800 bg-black p-3 font-mono text-xs text-emerald-200">
                {displaySnapshot.logs.map((line) => (
                  <div key={`${line.ts}-${line.message}`} className={clsx(line.level === 'error' && 'text-red-300', line.level === 'warn' && 'text-amber-200')}>
                    {line.ts} {line.message}
                  </div>
                ))}
              </div>
            </div>
          </Panel>
        </section>

        <section className="grid gap-4 xl:grid-cols-3">
          <Panel title="Dig / dump autonomy (mission)" icon={<GitBranch className="h-4 w-4 text-amber-300" />}>
            <DigDumpMissionChart current={displaySnapshot.mission.state} />
          </Panel>
          <Panel title="Dig sequence (node FSM)" icon={<Shovel className="h-4 w-4 text-amber-300" />}>
            <DigSequenceStateChart digSequence={displaySnapshot.mission.digSequence} />
          </Panel>
          <Panel title="Nav autonomy" icon={<Activity className="h-4 w-4 text-sky-300" />}>
            <NavAutonomyStateChart current={displaySnapshot.mission.state} />
          </Panel>
        </section>

        <section className="grid gap-4 xl:grid-cols-2">
          <Panel title="Localization And SLAM" icon={<Radio className="h-4 w-4 text-purple-300" />}>
            <div className="grid grid-cols-2 gap-2">
              <MetricCard
                metric={{
                  label: 'Pose source',
                  value: isNominalLikeScenario(scenario) ? '/odom' : 'none',
                  detail: isNominalLikeScenario(scenario) ? 'live topic' : 'audit needed',
                  status: isNominalLikeScenario(scenario) ? 'ok' : 'warn',
                }}
              />
              <MetricCard
                metric={{
                  label: 'Wheel odom',
                  value: isNominalLikeScenario(scenario) ? 'pending' : 'unknown',
                  detail: 'integrate encoders',
                  status: 'warn',
                }}
              />
              <MetricCard metric={{ label: 'Encoders', value: 'bad data', detail: 'needs fix', status: 'bad' }} />
              <MetricCard metric={{ label: 'SLAM', value: isNominalLikeScenario(scenario) ? 'evaluating' : 'not ready', detail: 'evaluate', status: 'warn' }} />
            </div>
          </Panel>
          <Panel title="Advanced Controls" icon={<Terminal className="h-4 w-4 text-slate-300" />}>
            <div className="grid grid-cols-2 gap-2">
              <Button disabled={!displaySnapshot.mission.connected}>Restart Mapper</Button>
              <Button disabled={!displaySnapshot.mission.connected}>Restart Bridge</Button>
              <Button disabled={!displaySnapshot.mission.connected}>Apply PID</Button>
              <Button>Open Foxglove</Button>
            </div>
            <div className="mt-3 rounded border border-amber-500/30 bg-amber-500/10 p-3 text-xs text-amber-100">
              Restart and tuning actions require confirmation before they affect the robot.
            </div>
          </Panel>
        </section>

        <section className="grid gap-4 xl:grid-cols-[1fr_1fr_1fr]">
          <Panel title="PID Tuning" icon={<SlidersHorizontal className="h-4 w-4 text-cyan-300" />}>
            <div className="space-y-3">
              {(['kp', 'ki', 'kd'] as const).map((key) => (
                <label key={key} className="block text-sm">
                  <div className="mb-1 flex justify-between text-xs uppercase text-slate-500">
                    <span>{key}</span>
                    <span>{pid[key].toFixed(2)}</span>
                  </div>
                  <input
                    type="range"
                    min="0"
                    max={key === 'kp' ? 10 : 5}
                    step={key === 'kp' ? 0.1 : 0.01}
                    value={pid[key]}
                    disabled={!displaySnapshot.mission.connected}
                    onChange={(event) => setPid((current) => ({ ...current, [key]: Number(event.target.value) }))}
                    className="w-full accent-cyan-400 disabled:opacity-40"
                  />
                </label>
              ))}
              <Button disabled={!displaySnapshot.mission.connected} onClick={() => void runCommand({ type: 'pid', gains: pid })}>Apply Gains</Button>
            </div>
          </Panel>

          <Panel title="Field Audit Checklist" icon={<ShieldAlert className="h-4 w-4 text-amber-300" />}>
            <div className="space-y-2">
              {displaySnapshot.audit.map((item) => (
                <div key={item.label} className={clsx('rounded border p-2 text-sm', statusClass(item.status))}>
                  <div className="flex items-center gap-2 font-semibold text-white">
                    <AlertDot status={item.status} />
                    {item.label}
                  </div>
                  <div className="mt-1 text-xs opacity-80">{item.detail}</div>
                </div>
              ))}
            </div>
          </Panel>

          <Panel title="Raw Config And Debug" icon={<Terminal className="h-4 w-4 text-slate-300" />}>
            <pre className="max-h-72 overflow-auto rounded border border-slate-800 bg-black p-3 text-xs text-emerald-200">
              {displaySnapshot.rawConfig}
            </pre>
          </Panel>
        </section>
      </main>
    </div>
  )
}

export default App
