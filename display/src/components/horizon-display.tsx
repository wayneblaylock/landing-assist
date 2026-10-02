import { useLayoutEffect, useRef, useState } from "react";

type HorizonState = {
  phiDeg: number;
  thetaDeg: number;
  cuePitchDeg: number;
  hFt: number;
  battPct: number;
  modelTUsable: boolean;
};

const INITIAL: HorizonState = {
  phiDeg: -16,
  thetaDeg: -6,
  cuePitchDeg: 1,
  hFt: 24,
  battPct: 100,
  modelTUsable: true,
};

const TAPE_TICKS = [50, 32, 16, 8, 4, 2, 1, 0];
const LADDER_DEG = [-40, -30, -20, -10, 10, 20, 30, 40];

const ARM = 78;
const HOOK = 26;
const INNER = 52;
const DOT_R = 4.2;
const YELLOW_STROKE = 6;
const MAGENTA_W = 86;
const MAGENTA_H = 8;
const CUE_SPAN = (INNER + ARM) * 2;

const LEFT_CUE = `M ${-(INNER + ARM)} 0 L ${-INNER} 0 L ${-INNER} ${HOOK}`;
const RIGHT_CUE = `M ${INNER} ${HOOK} L ${INNER} 0 L ${INNER + ARM} 0`;

function clamp(n: number, min: number, max: number) {
  return Math.min(max, Math.max(min, n));
}

function formatDeg(n: number) {
  return `${n.toFixed(1)}°`;
}

function formatFt(n: number) {
  const rounded = Math.round(n * 10) / 10;
  if (Math.abs(rounded - Math.round(rounded)) < 0.05) return String(Math.round(rounded));
  return rounded.toFixed(1);
}

function tapeWidthFor(w: number) {
  // Wide screens stay near the current ~100px scale. Narrow screens shrink
  // with width so the tape does not eat the horizon.
  return clamp(w * 0.072 + 24, 48, 112);
}

function tapeFontFor(tapeW: number) {
  return clamp(tapeW * 0.23, 11, 18);
}

/** 0 ft at the bottom, 50 ft at the top. ln(h+1) keeps 0 finite. */
function tapeT(ft: number) {
  const clamped = clamp(ft, 0, 50);
  return Math.log(clamped + 1) / Math.log(51);
}

export function HorizonDisplay() {
  const rootRef = useRef<HTMLDivElement>(null);
  const [box, setBox] = useState({ w: 0, h: 0 });
  const [state, setState] = useState<HorizonState>(INITIAL);
  const [devOpen, setDevOpen] = useState(false);

  useLayoutEffect(() => {
    const el = rootRef.current;
    if (!el) return;
    const measure = () => {
      const rect = el.getBoundingClientRect();
      setBox({ w: rect.width, h: rect.height });
    };
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(el);
    return () => observer.disconnect();
  }, []);

  const w = box.w;
  const h = box.h;
  const tapeW = w > 0 ? tapeWidthFor(w) : 72;
  const tapeFont = tapeFontFor(tapeW);
  const visibleW = Math.max(0, w - tapeW);
  const cx = visibleW / 2;
  const cy = h / 2;
  const pxPerDeg = Math.min(visibleW, h) / 34 || 1;
  const cueScale = Math.min((visibleW * 0.48) / CUE_SPAN, (h * 0.18) / HOOK, 2.6);
  const lineW = Math.max(1.75, Math.min(w, h) * 0.003);
  const padTop = Math.max(22, h * 0.04);
  const padBottom = Math.max(22, h * 0.045);
  const innerH = Math.max(1, h - padTop - padBottom);

  const yForFt = (ft: number) => padTop + (1 - tapeT(ft)) * innerH;
  const lowTop = yForFt(1);
  const lowHeight = Math.max(0, yForFt(0) - lowTop);

  // SVG applies the rightmost transform first: pitch slides the horizon
  // in its own vertical, then roll banks around the airplane symbol.
  // Positive pitch (nose up) moves the horizon down. Positive roll
  // (right wing down) uses -phi so the right side of the horizon rises.
  const worldTransform =
    w > 0
      ? `rotate(${-state.phiDeg} ${cx} ${cy}) translate(0 ${state.thetaDeg * pxPerDeg})`
      : undefined;

  const span = Math.max(w, h) * 3;
  const suggestionY = cy + (state.thetaDeg - state.cuePitchDeg) * pxPerDeg;

  const patch = (partial: Partial<HorizonState>) => setState((prev) => ({ ...prev, ...partial }));

  return (
    <div
      ref={rootRef}
      className="instrument"
      data-trust={state.modelTUsable ? "yes" : "no"}
      style={{ ["--tape-w" as string]: `${tapeW}px`, ["--tape-font" as string]: `${tapeFont}px` }}
    >
      {w > 0 && h > 0 && (
        <svg className="adi" width={w} height={h} aria-hidden="true">
          <defs>
            <clipPath id="adi-clip">
              <rect width={w} height={h} />
            </clipPath>
          </defs>
          <g clipPath="url(#adi-clip)">
            <g className="world" transform={worldTransform}>
              <rect className="sky" x={cx - span} y={cy - span} width={span * 2} height={span + 1} />
              <rect className="ground" x={cx - span} y={cy} width={span * 2} height={span} />
              {LADDER_DEG.map((deg) => {
                const y = cy - deg * pxPerDeg;
                return (
                  <line
                    key={deg}
                    className="ladder"
                    x1={cx - span}
                    y1={y}
                    x2={cx + span}
                    y2={y}
                    strokeWidth={lineW}
                  />
                );
              })}
            </g>
            <g className="suggestion" transform={`translate(${cx} ${suggestionY}) scale(${cueScale})`}>
              <rect
                className="aircraft"
                x={-MAGENTA_W / 2}
                y={-MAGENTA_H / 2}
                width={MAGENTA_W}
                height={MAGENTA_H}
              />
            </g>
            <g className="cue" transform={`translate(${cx} ${cy}) scale(${cueScale})`}>
              <path className="cue-stroke" d={LEFT_CUE} strokeWidth={YELLOW_STROKE} />
              <path className="cue-stroke" d={RIGHT_CUE} strokeWidth={YELLOW_STROKE} />
              <circle className="cue-dot" r={DOT_R} />
            </g>
          </g>
        </svg>
      )}

      <div className="tape" style={{ width: tapeW }}>
        <div className="low-band" style={{ top: lowTop, height: lowHeight }} />
        {TAPE_TICKS.map((ft) => (
          <div key={ft}>
            <span className="tape-tick" style={{ top: yForFt(ft) }} />
            <span className="tape-label" style={{ top: yForFt(ft) }}>
              {ft}
            </span>
          </div>
        ))}
        <div className="callout" style={{ top: yForFt(state.hFt) }}>
          <span className="callout-num">{formatFt(state.hFt)}</span>
          <span className="callout-arrow" />
        </div>
      </div>

      <div className="bottom-cluster">
        <div className="battery-wrap">
          <Battery percent={state.battPct} />
        </div>
        <div className="status" data-mode={state.modelTUsable ? "active" : "standby"} role="status">
          {state.modelTUsable ? "Active" : "Standby"}
        </div>
        <button
          type="button"
          className="dev-btn"
          aria-expanded={devOpen}
          onClick={() => setDevOpen((open) => !open)}
        >
          Dev
        </button>
      </div>

      {!state.modelTUsable && (
        <div className="trust-banner" role="alert">
          Don't Trust Sensors
        </div>
      )}

      {devOpen && (
        <aside className="dev-panel" aria-label="Sensor stand-in">
          <div className="dev-head">
            <h2 className="dev-title">Dev</h2>
            <button type="button" className="dev-close" onClick={() => setDevOpen(false)}>
              Close
            </button>
          </div>

          <label className="check-row">
            <input
              type="checkbox"
              checked={state.modelTUsable}
              onChange={(event) => patch({ modelTUsable: event.target.checked })}
            />
            <span>Model T</span>
            <span className="field-val">{state.modelTUsable ? "usable" : "dont_trust"}</span>
          </label>

          <Slider
            label="Roll"
            min={-60}
            max={60}
            step={0.1}
            value={state.phiDeg}
            display={formatDeg(state.phiDeg)}
            onChange={(phiDeg) => patch({ phiDeg })}
          />
          <Slider
            label="Actual pitch"
            min={-10}
            max={20}
            step={0.1}
            value={state.thetaDeg}
            display={formatDeg(state.thetaDeg)}
            onChange={(thetaDeg) => patch({ thetaDeg })}
          />
          <Slider
            label="Suggested pitch"
            min={-10}
            max={20}
            step={0.1}
            value={state.cuePitchDeg}
            display={formatDeg(state.cuePitchDeg)}
            onChange={(cuePitchDeg) => patch({ cuePitchDeg })}
          />
          <Slider
            label="Height"
            min={0}
            max={80}
            step={0.1}
            value={state.hFt}
            display={`${formatFt(state.hFt)} ft`}
            onChange={(hFt) => patch({ hFt })}
          />
          <Slider
            label="Battery"
            min={0}
            max={100}
            step={1}
            value={state.battPct}
            display={`${Math.round(state.battPct)}%`}
            onChange={(battPct) => patch({ battPct })}
          />

          <p className="dev-note">Positive roll is right wing down. Positive pitch is nose up.</p>
        </aside>
      )}
    </div>
  );
}

function Slider({
  label,
  min,
  max,
  step,
  value,
  display,
  onChange,
}: {
  label: string;
  min: number;
  max: number;
  step: number;
  value: number;
  display: string;
  onChange: (value: number) => void;
}) {
  return (
    <label className="field">
      <span className="field-label">
        <span>{label}</span>
        <span className="field-val">{display}</span>
      </span>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        aria-valuetext={display}
        onChange={(event) => onChange(Number(event.target.value))}
      />
    </label>
  );
}

function Battery({ percent }: { percent: number }) {
  const fill = clamp(percent, 0, 100) / 100;
  return (
    <svg className="battery-svg" viewBox="0 0 64 32" role="img" aria-label={`Battery ${Math.round(percent)} percent`}>
      <rect className="battery-body" x="2" y="2" width="52" height="28" rx="4" strokeWidth="3" />
      <rect className="battery-nub" x="56" y="10" width="6" height="12" rx="1.5" />
      <rect className="battery-fill" x="6" y="6" width={44 * fill} height="20" />
    </svg>
  );
}
