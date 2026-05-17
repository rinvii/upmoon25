import { clsx } from 'clsx'
import type { DigSequencePhase, DigSequenceSnapshot } from '../bridge/types'

const PIPELINE_WITH_ARM: DigSequencePhase[] = [
  'WAIT_NAV_ARM',
  'SETUP_IR',
  'DRIVE_FORWARD',
  'DRIVE_BACK',
  'CYCLE_END_CONVEYOR',
  'DONE',
]

const PIPELINE_NO_ARM: DigSequencePhase[] = [
  'SETUP_IR',
  'DRIVE_FORWARD',
  'DRIVE_BACK',
  'CYCLE_END_CONVEYOR',
  'DONE',
]

const SHORT_LABEL: Record<DigSequencePhase, string> = {
  WAIT_NAV_ARM: 'NAV',
  SETUP_IR: 'IR',
  DRIVE_FORWARD: 'FWD',
  DRIVE_BACK: 'BACK',
  CYCLE_END_CONVEYOR: 'CONV2',
  DONE: 'OK',
}

function nodeStyle(isCurrent: boolean, isPast: boolean) {
  if (isCurrent) return { fill: 'rgba(251, 191, 36, 0.22)', stroke: '#fbbf24', sw: 2.25 }
  if (isPast) return { fill: 'rgba(6, 78, 59, 0.45)', stroke: 'rgba(52, 211, 153, 0.55)', sw: 1.5 }
  return { fill: 'rgba(15, 23, 42, 0.92)', stroke: '#475569', sw: 1.25 }
}

const W = 920
const H = 200
const NODE_R = 19
const Y = 100
const MARGIN = 52

function pipelineXs(n: number): number[] {
  if (n <= 1) return [W / 2]
  const span = W - 2 * MARGIN
  return Array.from({ length: n }, (_, i) => MARGIN + (i * span) / (n - 1))
}

function isDigSequencePhase(s: string): s is DigSequencePhase {
  return (
    s === 'WAIT_NAV_ARM' ||
    s === 'SETUP_IR' ||
    s === 'DRIVE_FORWARD' ||
    s === 'DRIVE_BACK' ||
    s === 'CYCLE_END_CONVEYOR' ||
    s === 'DONE'
  )
}

/** Live FSM from ``dig_sequence`` via mission bridge (``/autonomy/dig_sequence/state`` → ``mission.digSequence``). */
export function DigSequenceStateChart({ digSequence }: { digSequence: DigSequenceSnapshot | null | undefined }) {
  const waitArm = Boolean(digSequence?.waitForNavDigArm)
  const flow = waitArm ? PIPELINE_WITH_ARM : PIPELINE_NO_ARM
  const phaseRaw = digSequence?.phase ?? ''
  const current: DigSequencePhase | null = isDigSequencePhase(phaseRaw) ? phaseRaw : null
  const pipeIdx = current !== null ? flow.indexOf(current) : -1
  const XS = pipelineXs(flow.length)

  return (
    <div className="space-y-4">
      <div className="flex flex-wrap items-center justify-between gap-2 text-[11px] text-slate-500">
        <span>
          Phase:{' '}
          <span className="font-mono text-amber-200">{digSequence?.phase ?? '—'}</span>
          {digSequence?.waitForNavDigArm ? (
            <span className="ml-2 rounded border border-amber-700/60 bg-amber-950/40 px-1.5 py-0.5 font-mono text-[10px] text-amber-100/90">
              nav arm gate
            </span>
          ) : null}
        </span>
        <span className="hidden sm:inline">Dig sequence · /autonomy/dig_sequence/state</span>
      </div>

      {!digSequence ? (
        <div className="rounded-lg border border-slate-800 bg-slate-950/50 p-4 text-sm text-slate-400">
          No dig sequence telemetry yet. Run <span className="font-mono text-slate-300">dig_sequence</span> (for example{' '}
          <span className="font-mono text-slate-300">lunar run nav-dig</span>) so the bridge can forward JSON on{' '}
          <span className="font-mono text-slate-300">/autonomy/dig_sequence/state</span>.
        </div>
      ) : (
        <div className="overflow-x-auto rounded-lg border border-slate-800 bg-slate-950/50 p-2 sm:p-4">
          <svg
            role="img"
            aria-label="Dig sequence controller pipeline"
            viewBox={`0 0 ${W} ${H}`}
            className="mx-auto h-auto w-full max-w-[min(100%,920px)]"
            preserveAspectRatio="xMidYMid meet"
          >
            <title>Dig sequence FSM</title>
            <defs>
              <marker id="dig-seq-arrow" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto" markerUnits="strokeWidth">
                <path d="M0,0 L7,3.5 L0,7 z" fill="#64748b" />
              </marker>
              <marker id="dig-seq-arrow-active" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto" markerUnits="strokeWidth">
                <path d="M0,0 L8,4 L0,8 z" fill="#fbbf24" />
              </marker>
            </defs>

            {flow.map((_, i) => {
              if (i >= flow.length - 1) return null
              const x1 = XS[i]! + NODE_R
              const x2 = XS[i + 1]! - NODE_R
              const active = pipeIdx === i
              return (
                <line
                  key={`dig-seq-e-${i}`}
                  x1={x1}
                  y1={Y}
                  x2={x2}
                  y2={Y}
                  stroke={active ? '#fbbf24' : '#475569'}
                  strokeWidth={active ? 2 : 1.25}
                  markerEnd={active ? 'url(#dig-seq-arrow-active)' : 'url(#dig-seq-arrow)'}
                />
              )
            })}

            {flow.map((id, i) => {
              const x = XS[i]!
              const isCurrent = current === id
              const isPast = pipeIdx !== -1 && i < pipeIdx
              const { fill, stroke, sw } = nodeStyle(isCurrent, isPast)
              return (
                <g key={id} transform={`translate(${x}, ${Y})`}>
                  <circle r={NODE_R} fill={fill} stroke={stroke} strokeWidth={sw} />
                  <text
                    textAnchor="middle"
                    dominantBaseline="central"
                    className={clsx(
                      'pointer-events-none select-none font-mono font-semibold',
                      isCurrent ? 'fill-amber-50' : isPast ? 'fill-emerald-100/90' : 'fill-slate-500',
                    )}
                    style={{ fontSize: 9 }}
                  >
                    {SHORT_LABEL[id]}
                  </text>
                  <title>{id}</title>
                </g>
              )
            })}

            <text x={W / 2} y={28} textAnchor="middle" className="fill-slate-600" style={{ fontSize: 10, fontFamily: 'ui-monospace, monospace' }}>
              dig sequence
            </text>
          </svg>
        </div>
      )}

      {digSequence ? (
        <div className="grid gap-2 rounded-md border border-slate-800 bg-slate-900/60 p-3 text-[11px] text-slate-400 sm:grid-cols-2">
          <div>
            <span className="text-slate-500">IR</span>{' '}
            <span className="font-mono text-slate-200">
              {digSequence.irValue ?? '—'} / {digSequence.irTarget ?? '—'}
            </span>
          </div>
          <div>
            <span className="text-slate-500">Encoder</span>{' '}
            <span className="font-mono text-slate-200">
              {digSequence.encoderValue ?? '—'} → {digSequence.encoderTarget ?? '—'}
            </span>
            {digSequence.encoderTopic ? (
              <span className="mt-0.5 block truncate font-mono text-[10px] text-slate-500">{digSequence.encoderTopic}</span>
            ) : null}
          </div>
          <div>
            <span className="text-slate-500">Cycles complete</span>{' '}
            <span className="font-mono text-slate-200">
              {digSequence.cycleCounter ?? '—'} / {digSequence.maxCyclesLe ?? '—'}
            </span>
          </div>
          <div>
            <span className="text-slate-500">Bucket cmd</span>{' '}
            <span className="font-mono text-slate-200">{digSequence.bucketPosCommanded ?? '—'}</span>
            {digSequence.keepBucketChainUntilDone ? (
              <span className="ml-2 text-amber-200/90">chain hold</span>
            ) : null}
          </div>
          <div>
            <span className="text-slate-500">Phase time</span>{' '}
            <span className="font-mono text-slate-200">
              {digSequence.phaseElapsedSec != null ? `${digSequence.phaseElapsedSec.toFixed(1)} s` : '—'}
            </span>
          </div>
          <div>
            <span className="text-slate-500">Conveyor</span> <span className="font-mono text-slate-200">disabled</span>
          </div>
          {digSequence.waitForNavDigArm ? (
            <div className="sm:col-span-2">
              <span className="text-slate-500">/autonomy/dig_arm</span>{' '}
              <span className={clsx('font-mono', digSequence.digArm ? 'text-emerald-300' : 'text-slate-500')}>
                {digSequence.digArm ? 'true' : 'false'}
              </span>
            </div>
          ) : null}
          {digSequence.useLocalTerrainGrid ? (
            <div className="sm:col-span-2 rounded border border-emerald-900/40 bg-emerald-950/20 px-2 py-1.5">
              <div className="text-[10px] font-semibold uppercase tracking-wide text-emerald-200/80">Local terrain (nav map)</div>
              <div className="mt-1 flex flex-wrap gap-x-3 gap-y-1 text-[11px] text-slate-400">
                <span>
                  grid{' '}
                  <span className={clsx('font-mono', digSequence.terrainHadGrid ? 'text-slate-200' : 'text-slate-500')}>
                    {digSequence.terrainHadGrid ? 'live' : 'waiting'}
                  </span>
                </span>
                <span>
                  fresh{' '}
                  <span className={clsx('font-mono', digSequence.terrainFresh ? 'text-emerald-300' : 'text-amber-200')}>
                    {digSequence.terrainFresh ? 'yes' : 'no'}
                  </span>
                </span>
                <span>
                  fwd{' '}
                  <span className={clsx('font-mono', digSequence.terrainForwardOk !== false ? 'text-emerald-300' : 'text-red-300')}>
                    {digSequence.terrainForwardOk !== false ? 'ok' : 'hold'}
                  </span>
                </span>
                <span>
                  rev{' '}
                  <span className={clsx('font-mono', digSequence.terrainReverseOk !== false ? 'text-emerald-300' : 'text-red-300')}>
                    {digSequence.terrainReverseOk !== false ? 'ok' : 'hold'}
                  </span>
                </span>
              </div>
              {(digSequence.terrainGateForward || digSequence.terrainGateReverse) && (
                <div className="mt-1 font-mono text-[10px] text-slate-500">
                  {digSequence.terrainGateForward ?? '—'} · {digSequence.terrainGateReverse ?? '—'}
                </div>
              )}
            </div>
          ) : null}
        </div>
      ) : null}

      <p className="text-[11px] leading-relaxed text-slate-500">
        This chart tracks the <span className="font-mono text-slate-400">dig_sequence</span> node (IR setup, encoder-scoped drive, conveyor cycles). When enabled, drive holds use the same{' '}
        <span className="font-mono text-slate-400">/autonomy/local_terrain_grid</span> as nav. This is separate from the high-level{' '}
        <span className="font-mono text-slate-400">mission.state</span> dig/dump diagram.
      </p>
    </div>
  )
}
