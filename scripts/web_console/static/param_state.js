// 파라미터 상태: /api/params 스냅샷(그룹·차량 특성 탭 구성, 현재값, 기본값, 항목 설명)과 값 바꾸기. 파라미터 탭과
// 차량 특성 탭이 같이 쓴다.
import {api} from "./ui.js";

let snapshot = null;

export async function loadParams() {
  snapshot = await api.get("/api/params");
  return snapshot;
}

export function params() {
  return snapshot;
}

// 설명이 없는 항목(파일에는 있고 메타데이터에는 없는 키)의 기본 설명
export function metaFor(group, key) {
  const meta = snapshot.metadata[group]?.[key];
  if (meta) return meta;
  const value = snapshot.params[group][key];
  return {
    label: key, section: "기타", unit: "", step: Number.isInteger(value) ? 1 : 0.01, min: -1e6, max: 1e6,
    description: "설명이 등록되지 않은 파라미터입니다.", increase: "값이 커집니다.", decrease: "값이 작아집니다.",
  };
}

export function valueOf(group, key) {
  return snapshot.params[group][key];
}

export function defaultOf(group, key) {
  return snapshot.defaults?.[group]?.[key];
}

export function isModified(group, key) {
  const fallback = defaultOf(group, key);
  return fallback !== undefined && JSON.stringify(fallback) !== JSON.stringify(valueOf(group, key));
}

function decimals(step) {
  const text = String(step);
  if (text.includes("e-")) return Number(text.split("e-")[1]);
  return text.includes(".") ? text.split(".")[1].length : 0;
}

// 범위 안으로 자르고 step의 자릿수(minDigits가 더 크면 그 자릿수)로 반올림
export function clampToMeta(value, meta, minDigits = 0) {
  return Number(Math.min(meta.max, Math.max(meta.min, value)).toFixed(Math.max(decimals(meta.step), minDigits)));
}

/* 값 하나를 저장하고 스냅샷을 갱신한다. {applied, notified}를 돌려주고, 실패하면 예외를 던진다. */
export async function saveParam(group, key, value) {
  const update = await api.patch(`/api/params/${group}`, {values: {[key]: value}});
  snapshot.params[group] = update.params;
  return {applied: update.params[key], notified: update.notified || []};
}
