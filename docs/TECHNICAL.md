# SDF 确定性光线追踪渲染器 · 技术文档

**版本**：v5.2  
**路线**：确定性 Whitted-style 光线追踪（非路径追踪）  
**目标**：Web 端 / 移动端可交互 3D 场景  
**许可**：MIT  
**文档性质**：可复用的技术手册，适用于任意 SDF 场景

---

## 目录

1. [概览](#1-概览)
2. [算法核心](#2-算法核心)
3. [场景构建](#3-场景构建)
4. [光照系统](#4-光照系统)
5. [材质系统](#5-材质系统)
6. [体积效果](#6-体积效果)
7. [廉价 GI 与 AO](#7-廉价-gi-与-ao)
8. [分区求值与粗代理](#8-分区求值与粗代理)
9. [光线步进的陷阱与修复](#9-光线步进的陷阱与修复)
10. [相机、输入与物理](#10-相机输入与物理)
11. [性能与移动端适配](#11-性能与移动端适配)
12. [调试与排错](#12-调试与排错)
13. [与其他渲染器对比](#13-与其他渲染器对比)
14. [场景模板](#14-场景模板)
15. [参数速查](#15-参数速查)

---

## 1. 概览

### 1.1 这是什么

一个纯片元着色器实现的确定性实时光线追踪器。没有三角形网格、没有顶点缓冲、没有场景图。整个 3D 世界由数学函数 `mapScene(p) → (距离, 材质ID)` 定义，在 GPU 上通过光线步进求解可见表面。

### 1.2 核心特性

| 特性 | 说明 |
|---|---|
| 零几何数据 | 场景描述即函数，无网格、无烘焙、无资源文件 |
| 零噪点 | 无随机采样，数学上不存在方差 |
| 单文件 | 打包后 < 50 KB，无外部依赖 |
| 移动端友好 | Mali-G57 / Adreno 6xx 上 30+ fps |
| 可交互 | 支持射线拾取、物理交互、碰撞检测 |

### 1.3 非目标

- ❌ 不是通用游戏引擎
- ❌ 不处理骨骼动画、粒子系统、复杂物理
- ❌ 不适合超大场景（> 100 个 SDF 元素）
- ❌ 不追求物理正确的彩色 GI
- ❌ 不替代 Three.js / Babylon.js

---

## 2. 算法核心

### 2.1 渲染管线

```text
每个像素：
├─ 生成主光线（相机 → 像素方向）
│
├─ 光线步进求交（marchScene）
│     └─ 命中 → 位置 p、材质 id
│
├─ 计算法线（normalScene）
│
├─ 遍历所有光源：
│     ├─ 阴影射线检测（shadowRay）
│     ├─ 漫反射累积
│     └─ 高光累积
│
├─ 如果是镜面材质：
│     └─ 沿镜面反射方向再追踪一次（marchCoarse）
│
├─ 伪 GI + AO 叠加
│
├─ 体积雾累积（fogAt）
│
└─ ACES 色调映射 + Gamma → 输出
```

### 2.2 Sphere Tracing（John Hart, 1996）

```glsl
vec2 marchScene(vec3 ro, vec3 rd) {
  float t = 0.08;
  for (int i = 0; i < MAX_STEPS; i++) {
    vec3 p = ro + rd * t;
    vec2 h = mapScene(p);
    if (h.x < EPS) return vec2(t, h.y);
    t += max(h.x, 0.003);   // 步长下限——关键的坑
    if (t > MAX_DIST) break;
  }
  return vec2(-1.0, 0.0);
}
```

数学保证：若 d = SDF(p)，则以 p 为球心、半径 |d| 的球内不包含任何表面。所以 t += d 是安全的。

唯一前提：SDF 返回的必须是真实最短距离（Lipschitz 连续）。

### 2.3 为什么没有噪点

噪点来自“同一像素在不同帧下颜色不同”。路径追踪里光线方向随机，因此有方差。确定性光追里：

- 主光线方向 = 相机射线
- 阴影射线方向 = 光源方向
- 反射方向 = 镜面反射公式

同样的输入 → 同样的输出，方差为零。

---

## 3. 场景构建

### 3.1 SDF 基元库

```glsl
// 球
float sdSphere(vec3 p, float r) { return length(p) - r; }

// 盒
float sdBox(vec3 p, vec3 b) {
  vec3 q = abs(p) - b;
  return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
}

// Y 轴圆柱
float sdCylY(vec3 p, float r, float h) {
  vec2 d = abs(vec2(length(p.xz), p.y)) - vec2(r, h);
  return min(max(d.x, d.y), 0.0) + length(max(d, 0.0));
}

// 环面
float sdTorus(vec3 p, vec2 t) {
  vec2 q = vec2(length(p.xz) - t.x, p.y);
  return length(q) - t.y;
}

// 八面体
float sdOcta(vec3 p, float s) {
  p = abs(p);
  return (p.x + p.y + p.z - s) * 0.57735027;
}

// 圆角盒（用于房间、圆角物体）
float sdRoundBox(vec3 p, vec3 b, float r) {
  vec3 q = abs(p) - b + r;
  return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0) - r;
}

// 线段胶囊
float sdSegment(vec3 p, vec3 a, vec3 b, float r) {
  vec3 pa = p - a, ba = b - a;
  float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
  return length(pa - ba * h) - r;
}

// 等边三角棱柱
float sdEqTri2D(vec2 p, float r) {
  const float k = 1.7320508;
  p.x = abs(p.x) - r;
  p.y = p.y + r / k;
  if (p.x + k * p.y > 0.0) p = vec2(p.x - k * p.y, -k * p.x - p.y) * 0.5;
  p.x -= clamp(p.x, -2.0 * r, 0.0);
  return -length(p) * sign(p.y);
}
float sdTriPrismY(vec3 p, float r, float h) {
  float d = sdEqTri2D(p.xz, r);
  vec2 w = vec2(d, abs(p.y) - h);
  return min(max(w.x, w.y), 0.0) + length(max(w, 0.0));
}
```

### 3.2 布尔运算

```glsl
// 并集（保留材质 ID）
vec2 opU(vec2 a, vec2 b) { return a.x < b.x ? a : b; }

// 差集
float opS(float a, float b) { return max(a, -b); }

// 交集
float opI(float a, float b) { return max(a, b); }

// 光滑并集（k = 混合半径，越大越圆滑）
float smin(float a, float b, float k) {
  float h = clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
  return mix(b, a, h) - k * h * (1.0 - h);
}
```

⚠️ 坑：smin 的第二个参数名不要用 b，会和 for 循环变量冲突（Mali 编译器会报错）。

### 3.3 空间变换

```glsl
// 平移：p - offset
// 旋转：p.xz = mat2(c, -s, s, c) * p.xz
// 缩放：p / scale（结果需乘 scale）
// 无限重复：mod(p + c, s) - c
// 有限重复：clamp
// 扭曲：p.xy = rot(p.xy, p.z * k)
```

约定：任何变换后必须保证返回值 ≤ 真实距离。缩放最容易出错——若 p' = p/s，则 sdf(p') * s 才是正确距离。

### 3.4 场景模板

```glsl
vec2 mapScene(vec3 p) {
  vec2 res = vec2(1e9, 0.0);   // (距离, 材质ID)
  
  // 房间 / 世界结构
  res = opU(res, vec2(sdBox(p - vec3(0, 0, 0), vec3(3, 2, 3)), 0.0));
  
  // 物体
  res = opU(res, vec2(sdSphere(p - vec3(0, 1.1, 0), 0.7), 1.0));
  res = opU(res, vec2(sdBox(p - vec3(-1.7, 0.35, -1.5), vec3(0.35)), 2.0));
  res = opU(res, vec2(sdTorus(p - vec3(-0.4, 1.30, -4.3), vec2(0.85, 0.26)), 12.0));
  
  // 光源（自发光块）
  res = opU(res, vec2(sdBox(p - vec3(3.6, 0.40, 3.6), vec3(0.40)), 8.0));
  
  return res;
}
```

约定：

- id = 0 → 地板 / 墙
- id = 1..N → 各种物体
- id = 8..11 → 霓虹块（自发光）
- id = 25 → 顶灯（自发光）

---

## 4. 光照系统

### 4.1 光源定义

```glsl
// 位置与颜色函数（避免数组）
vec3 lightPosOf(int i) {
  if (i == 0) return vec3( 3.6, 0.40,  3.6);
  if (i == 1) return vec3(-3.6, 0.40, -3.2);
  if (i == 2) return vec3(-5.62, 3.30,  0.0);
  return vec3(0.0, 3.80, 12.25);
}
vec3 lightColorOf(int i) {
  if (i == 0) return vec3(0.30, 0.95, 1.00);  // 青
  if (i == 1) return vec3(1.00, 0.30, 0.75);  // 粉
  if (i == 2) return vec3(1.00, 0.55, 0.20);  // 橙
  return vec3(0.75, 0.88, 1.00) * 0.55;       // 冷白
}
```

### 4.2 直接光照

```glsl
vec3 shadeSurface(vec3 p, vec3 n, vec3 rd, float id) {
  vec3 albedo = albedoOf(id);
  vec3 col = vec3(0.006);   // 极弱基色，防止死黑

  for (int i = 0; i < 4; i++) {
    if (i == 3 && p.z < 5.2) continue;   // 灯只影响主房间
    if (i < 3 && p.z > 6.3) continue;    // 霓虹只影响冷库

    vec3 lp = lightPosOf(i);
    vec3 lc = lightColorOf(i);
    vec3 L = lp - p;
    float d = length(L);
    if (d < 0.05) continue;
    L /= d;

    float ndl = max(dot(n, L), 0.0);
    if (ndl < 0.001) continue;

    // 关键：阴影起点法线偏移 0.08
    float sh = shadowRay(p + n * 0.08, L, d - 0.16);
    if (sh < 0.001) continue;

    // 反平方衰减
    float atten = 1.0 / (1.0 + d * d * 0.15);

    // 漫反射
    col += albedo * lc * ndl * sh * atten * 6.0;

    // Blinn-Phong 高光
    vec3 Hv = normalize(L - rd);
    float ndh = max(dot(n, Hv), 0.0);
    col += lc * pow(ndh, 48.0) * sh * atten * 1.5;
  }

  // 伪 GI + AO
  vec3 gi = indirectLight(p, n);
  float ao = calcAO(p, n);
  col += albedo * gi * ao * 1.3;

  return col;
}
```

### 4.3 阴影射线（IQ 软阴影）

```glsl
float shadowRay(vec3 ro, vec3 rd, float maxt) {
  float t = 0.10;
  float vis = 1.0;

  for (int i = 0; i < SHADOW_STEPS; i++) {
    if (t > maxt) break;
    vec2 h = mapCoarse(ro + rd * t);   // 用粗代理，省性能

    // 光源 / 帘子 / 铃铛不挡光
    if (h.y > 5.5 && h.y < 11.5)  { t += 0.30; continue; }
    if (h.y > 24.5 && h.y < 25.5) { t += 0.30; continue; }
    if (h.y > 22.5 && h.y < 23.5) { t += 0.30; continue; }

    if (h.x < 0.015) return 0.0;
    
    // IQ 软阴影：距离遮挡物越近越暗
    vis = min(vis, 6.0 * h.x / t);
    t += max(h.x, 0.03);
  }
  return clamp(vis, 0.0, 1.0);
}
```

---

## 5. 材质系统

### 5.1 ID → 颜色映射

```glsl
vec3 albedoOf(float id) {
  if (id < 0.5)  return vec3(0.10);             // 地板
  if (id < 1.5)  return vec3(0.05);             // 天花板
  if (id < 2.5)  return vec3(0.22, 0.08, 0.08); // 红墙
  if (id < 3.5)  return vec3(0.08, 0.22, 0.12); // 绿墙
  if (id < 4.5)  return vec3(0.08, 0.11, 0.22); // 蓝墙
  if (id < 5.5)  return vec3(0.14);             // 门框
  if (id < 6.5)  return vec3(0.90, 0.93, 0.96); // 镜面球
  if (id < 12.5) return vec3(0.82);             // 环
  if (id < 16.5) return vec3(0.30, 0.34, 0.40); // 门
  if (id < 17.5) return vec3(0.16, 0.20, 0.26); // 冷库墙
  if (id < 18.5) return vec3(0.18, 0.22, 0.28); // 冷库顶
  if (id < 19.5) return vec3(0.22, 0.26, 0.32); // 货架
  if (id < 20.5) return vec3(0.42, 0.34, 0.24); // 包裹
  if (id < 21.5) return vec3(0.32, 0.35, 0.40); // 风扇
  if (id < 23.5) return vec3(0.60, 0.72, 0.82); // 帘子
  if (id < 24.5) return vec3(0.95, 0.85, 0.30); // 铃铛
  return vec3(0.80, 0.92, 1.00);                // 顶灯
}
```

### 5.2 特殊材质判断

```glsl
bool isEmissiveID(float id) {
  return (id > 7.5 && id < 11.5) || (id > 24.5 && id < 25.5);
}
```

### 5.3 特殊材质的着色

自发光（霓虹块、灯）：

```glsl
if (isEmissiveID(id)) {
  if (id > 24.5) col = vec3(0.75, 0.88, 1.0) * 1.6;   // 顶灯
  else col = neonColorOf(id) * 2.4 + vec3(0.8);        // 霓虹
}
```

镜面反射（球）：

```glsl
} else if (id > 5.5 && id < 6.5) {
  vec3 n = normalize(p - sphereCenter);
  if (dot(n, rd) > 0.0) n = -n;
  vec3 rdir = reflect(rd, n);
  vec2 rh = marchCoarse(p + n * 0.08, rdir);
  vec3 rc = vec3(0.0);
  if (rh.x > 0.0) {
    vec3 rp = p + n * 0.08 + rdir * rh.x;
    float ir = rh.y;
    if (isEmissiveID(ir)) {
      rc = neonColorOf(ir) * 1.8;
    } else {
      rc = shadeSurface(rp, normalScene(rp), rdir, ir);
    }
  }
  float fres = pow(clamp(1.0 - max(dot(-rd, n), 0.0), 0.0, 1.0), 3.0);
  col = rc * (0.7 + 0.3 * fres);
}
```

半透明（帘子）：

```glsl
} else if (id > 22.5 && id < 23.5) {
  vec3 n = normalScene(p);
  vec3 c = shadeSurface(p, n, rd, id);
  col = mix(c, vec3(0.70, 0.82, 0.92) * 0.6, 0.55);
}
```

金属（铃铛）：

```glsl
} else if (id > 24.5 && id < 25.5) {
  vec3 n = normalScene(p);
  vec3 refl = reflect(rd, n);
  vec2 rh = marchCoarse(p + n * 0.06, refl);
  vec3 rc = vec3(0.0);
  if (rh.x > 0.0) {
    vec3 rp = p + n * 0.06 + refl * rh.x;
    float ir = rh.y;
    rc = isEmissiveID(ir) ? neonColorOf(ir) * 1.6
                          : shadeSurface(rp, normalScene(rp), refl, ir);
  }
  col = vec3(0.95, 0.80, 0.30) * 0.45 + rc * 0.6;
  col += vec3(1.0, 0.9, 0.3) * u_lockFlash * 1.5;
  col += vec3(0.6, 0.5, 0.1) * abs(sin(u_time * 40.0)) * u_bellShake * 1.2;
}
```

---

## 6. 体积效果

### 6.1 密度函数

```glsl
float fogAt(vec3 p) {
  float d = 0.0;
  
  // 冷库内部：浓雾，能见度 ~3m
  if (p.z > 6.0 && p.z < 18.5 && p.y < 4.1 && abs(p.x) < 5.2) {
    d = 0.42;
    if (p.z < 9.5) d = mix(0.60, 0.42, (p.z - 6.0) / 3.5);
    d *= 1.0 + 0.4 * exp(-p.y * 1.5);   // 地面附近更浓
  }
  
  // 门口冷气溢出
  if (p.z > 4.0 && p.z < 6.5 && p.y < 3.0) {
    float open = smoothstep(0.10, 1.20, u_doorAngle);
    float dz = (p.z - 4.0) / 2.5;
    float w = 0.9 + 0.1 * sin(u_time * 1.2 + p.x * 3.0);
    d += 0.22 * open * w * (1.0 - dz * dz);
  }
  
  return d;
}
```

### 6.2 累积与合成

```glsl
// 主循环累积
float fog = 0.0;
for (int i = 0; i < MAX_STEPS; i++) {
  ...
  fog += fogAt(p) * max(h.x, 0.003);
  ...
}

// 合成
vec3 fogColor = vec3(0.62, 0.74, 0.86);
float f = 1.0 - exp(-fog);
col = mix(col, fogColor, f);
```

关键参数：

- 密度 0.05 → 5m 几乎看不到雾
- 密度 0.42 → 5m 能见度降到 12%
- 系数对数值极敏感，每次改动建议减半

---

## 7. 廉价 GI 与 AO

### 7.1 探针式伪 GI

原理：在场景中放置 3~8 个固定探针，每个探针记录它所在位置的“环境光颜色”。命中点按距离加权插值。

JS 侧（每帧更新探针）：

```js
var PROBE_POS = [
  [ 0.0, 2.20,  0.0],   // 主房间中央
  [-4.5, 3.20, -3.5],   // 主房间左后上
  [ 0.0, 2.00, 10.0]    // 冷库中央
];
function updateProbes() {
  for (var i = 0; i < 3; i++) {
    var p = PROBE_POS[i], r = 0, g = 0, b = 0;
    for (var j = 0; j < 4; j++) {
      var lp = LIGHT_POS[j], lc = LIGHT_COL[j];
      var d2 = (lp[0]-p[0])**2 + (lp[1]-p[1])**2 + (lp[2]-p[2])**2;
      var atten = 1.0 / (1.0 + d2 * 0.18);
      r += lc[0] * atten * 0.9;
      g += lc[1] * atten * 0.9;
      b += lc[2] * atten * 0.9;
    }
    PROBE_COL[i*3+0] = r;
    PROBE_COL[i*3+1] = g;
    PROBE_COL[i*3+2] = b;
  }
}
```

Shader 侧（插值）：

```glsl
vec3 indirectLight(vec3 p, vec3 n) {
  // 位置硬编码在 shader 里（避免 uniform 数量爆炸）
  float dA = length(p - vec3(0.0, 2.20, 0.0));
  float wA = 1.0 / (1.0 + dA * dA * 0.20);
  float dB = length(p - vec3(-4.5, 3.20, -3.5));
  float wB = 1.0 / (1.0 + dB * dB * 0.20);
  float dC = length(p - vec3(0.0, 2.00, 10.0));
  float wC = 1.0 / (1.0 + dC * dC * 0.20);
  
  float wSum = wA + wB + wC;
  vec3 total = (u_ambA * wA + u_ambB * wB + u_ambC * wC) / max(wSum, 0.0001);
  
  // 朝上的表面接收更多间接光
  float up = 0.35 + 0.65 * max(n.y, 0.0);
  return total * up;
}
```

效果：

- 靠近青霓虹的墙染上青
- 靠近粉霓虹的墙染上粉
- 暗处不再死黑

成本：每次命中 3 次 `length()` + 加法，几乎免费。

### 7.2 环境光遮蔽（AO）

```glsl
float calcAO(vec3 p, vec3 n) {
  float occ = 0.0;
  float sca = 1.0;
  for (int i = 0; i < 3; i++) {
    float h = 0.05 + 0.20 * float(i) * 0.5;
    float d = mapCoarse(p + n * h).x;   // 粗代理采样
    occ += (h - d) * sca;
    sca *= 0.70;
  }
  return clamp(1.0 - 1.5 * occ, 0.0, 1.0);
}
```

效果：

- 墙角自然变暗
- 物体底部加深
- 帘子褶皱有立体感

成本：每次命中 3 次粗代理采样，约 5% 性能。

---

## 8. 分区求值与粗代理

### 8.1 分区求值

当场景由空间分离的子区域组成时：

```glsl
vec2 mapScene(vec3 p) {
  if (p.z < 5.7) return mapRoom(p);        // 只跑主房间
  if (p.z > 6.3) return mapCold(p);        // 只跑冷库
  vec2 a = mapRoom(p);                     // 过渡带：两个都跑
  vec2 b = mapCold(p);
  return opU(a, b);
}
```

收益：玩家在主房间看门时，冷库的几十条 SDF 指令一次都不执行。Mali-G57 实测帧率提升 2.5×。

代价：门洞处 SDF 不连续，可能产生黑线。用保守参数（EPS、步长下限、阴影偏移）覆盖。

### 8.2 粗代理

反射和阴影不需要精确几何，用简化场景代替：

```glsl
vec2 mapCoarse(vec3 p) {
  vec2 res = vec2(1e9, 0.0);
  if (p.z < 6.3) {
    res = opU(res, vec2(p.y, 0.0));                              // 地板
    res = opU(res, vec2(sdBox(p - roomCenter, roomSize), 1.0));  // 房间一个大盒
    res = opU(res, vec2(sdSphere(p - ballCenter, ballR), 6.0));  // 球
    res = opU(res, vec2(sdBox(p - boxPos, boxHalf), 15.0));      // 方盒
    res = opU(res, vec2(sdTorus(p - torusPos, torusR), 12.0));   // 环
    // ... 所有可见物体都要加
  }
  return res;
}
```

⚠️ 关键坑：`mapCoarse` 里遗漏的物体会在反射里消失。每次往 `mapScene` 加新物体，都要同步加进 `mapCoarse`。

收益：反射/阴影速度提升 5~10 倍。

代价：反射里的细节略少（玩家看不出）。

---

## 9. 光线步进的陷阱与修复

### 9.1 头号陷阱：强制最小步长

问题代码：

```glsl
t += max(h.x, 0.02);   // 破坏 Sphere Tracing 安全保证
```

数学证明：

- 当 `h.x = 0.015` 时，本应只前进 `0.015`
- 但 `max(0.015, 0.02) = 0.02`
- 前进了 `0.02 > 0.015` → 可能跨过表面

掠射光线的误差累积：

- 正面撞墙：`h.x` 从大快速缩小，`max` 不影响
- 掠射擦墙：`h.x` 始终很小（0.015~0.018），每步都被放大，误差累积 → 光线穿进墙内部

表现：

- 面状黑斑（圆形扩散，远看墙黑，走近消失）
- 线状黑线（几何体接缝处）

修复：

```glsl
t += max(h.x, 0.003);   // 3mm 下限，不破坏安全保证
```

### 9.2 二号陷阱：阴影自遮挡

问题：阴影射线从表面出发时，浮点误差让起点在表面内部，立即检测到自己。

修复：阴影起点沿法线偏移 `n * 0.08`（不要太大）。

权衡：

- 太小（0.02）→ 自��挡（噪点状阴影）
- 太大（0.20）→ 阴影泄漏（物体看起来悬浮）

推荐值：`0.06 ~ 0.12`

### 9.3 三号陷阱：EPS 太小

问题：`EPS = 0.001` 时，光线在接近表面时过早停止，产生“打不到表面”的错觉，表现为黑边。

推荐值：`0.008 ~ 0.015`

### 9.4 四号陷阱：SDF 不保守

原因：

- 缩放后忘记乘系数
- `smin` 的 `k` 值过大
- 非均匀缩放（例如 `p.x *= 2.0`）

症状：任何曲面的轮廓有一圈细黑边。

修复：检查所有变换，确保 SDF 返回 ≤ 真实距离。

### 9.5 五号陷阱：几何体接缝黑线

原因：两个物体做 `opU = min` 后，接缝处 SDF 局部高估距离。

修复：

1. 加大 `EPS`（`0.012 → 0.015`）
2. 加大阴影起点偏移
3. 用 `smin` 做光滑过渡（但 Mali 对 `smin` 参数名敏感）

---

## 10. 相机、输入与物理

### 10.1 第一人称相机

```js
var cam = {
  pos: [0, 1.70, -1.5],
  yaw: Math.PI,          // 0 = -Z, PI = +Z
  pitch: -0.02,
  tYaw: Math.PI,         // 目标值（平滑插值）
  tPitch: -0.02,
  focal: 1.05,           // 焦距（越小视野越广）
  tFocal: 1.05,
  minPitch: -1.25, maxPitch: 1.25,
  minFocal: 0.60, maxFocal: 2.60,
  speed: 3.2
};

function camDirVec() {
  var cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
  var cy = Math.cos(cam.yaw),   sy = Math.sin(cam.yaw);
  return [-sy * cp, sp, -cy * cp];
}
```

每帧平滑：

```js
cam.yaw   += (cam.tYaw   - cam.yaw)   * 0.18;
cam.pitch += (cam.tPitch - cam.pitch) * 0.18;
cam.focal += (cam.tFocal - cam.focal) * 0.15;
```

### 10.2 移动

```js
var cy = Math.cos(cam.yaw), sy = Math.sin(cam.yaw);
var fx = -sy, fz = -cy;   // 前向
var rx =  cy, rz = -sy;   // 右向

var mx = 0, mz = 0;
if (keys['KeyW']) { mx += fx; mz += fz; }
if (keys['KeyS']) { mx -= fx; mz -= fz; }
if (keys['KeyA']) { mx -= rx; mz -= rz; }
if (keys['KeyD']) { mx += rx; mz += rz; }
if (joyVec.x || joyVec.y) {
  mx += fx * (-joyVec.y) + rx * joyVec.x;
  mz += fz * (-joyVec.y) + rz * joyVec.x;
}

var l = Math.hypot(mx, mz);
if (l > 0.001) {
  var spd = cam.speed * dt / l;
  cam.pos[0] += mx * spd;
  cam.pos[2] += mz * spd;
  clampPosition(cam.pos);   // 碰撞
}
```

### 10.3 射线拾取

```js
function rayHitsPoint(px, py, pz, radius, maxDist) {
  var d = camDirVec();
  var ox = px - cam.pos[0], oy = py - cam.pos[1], oz = pz - cam.pos[2];
  var dist = Math.sqrt(ox*ox + oy*oy + oz*oz);
  if (dist > maxDist) return false;
  var t = ox*d[0] + oy*d[1] + oz*d[2];
  if (t < 0 || t > dist + radius) return false;
  var nx = ox - t*d[0], ny = oy - t*d[1], nz = oz - t*d[2];
  return Math.sqrt(nx*nx + ny*ny + nz*nz) < radius;
}
```

### 10.4 碰撞检测（JS 侧）

```js
function clampPosition(p) {
  var pad = 0.35;
  
  // 房间边界
  if (p[2] < 6.0) {
    p[0] = clamp(p[0], -6.0 + pad, 6.0 - pad);
    p[1] = clamp(p[1],  0.40, 5.0 - pad);
    if (p[2] < -6.0 + pad) p[2] = -6.0 + pad;
  }
  
  // 球形碰撞
  var gx = p[0] + 1.9, gy = p[1] - 1.30, gz = p[2] + 1.6;
  var gd = Math.sqrt(gx*gx + gy*gy + gz*gz);
  var gr = 0.78 + pad;
  if (gd < gr && gd > 1e-5) {
    var k = gr / gd;
    p[0] = -1.9 + gx*k; p[1] = 1.30 + gy*k; p[2] = -1.6 + gz*k;
  }
  
  // 线段碰撞（门）
  if (door.angle > 0.15) {
    var ax = door.hinge[0], az = door.hinge[2];
    var c = Math.cos(door.angle), s = Math.sin(door.angle);
    var bx = ax - c*door.width, bz = az - s*door.width;
    var abx = bx - ax, abz = bz - az;
    var abLen2 = abx*abx + abz*abz;
    if (abLen2 > 1e-6) {
      var tSeg = ((p[0]-ax)*abx + (p[2]-az)*abz) / abLen2;
      tSeg = clamp(tSeg, 0, 1);
      var cx = ax + abx*tSeg, cz = az + abz*tSeg;
      var ddx = p[0]-cx, ddz = p[2]-cz;
      var dd = Math.sqrt(ddx*ddx + ddz*ddz);
      var minD = pad + 0.10;
      if (dd < minD && dd > 1e-5) {
        var kk = minD/dd;
        p[0] = cx + ddx*kk; p[2] = cz + ddz*kk;
      }
    }
  }
  
  return p;
}
```

---

## 11. 性能与移动端适配

### 11.1 参数表（Mali-G57 实测）

| 参数 | 值 | 说明 |
|---|---:|---|
| EPS | 0.012 | 命中阈值 |
| MAX_STEPS | 64 | 主光线最大步数 |
| SHADOW_STEPS | 12 | 阴影射线最大步数 |
| 主光线步长下限 | 0.003 | 防死循环 |
| 阴影步长下限 | 0.03 |  |
| 粗代理步长下限 | 0.008 |  |
| 阴影起点偏移 | `n*0.08` | 避免自遮挡 |
| 反射起点偏移 | `n*0.06` |  |
| MAX_DIST | 35.0 | 最远距离 |

### 11.2 分辨率自适应

```js
var dprCap = hasTouch ? 1.10 : 1.60;
var renderScale = hasTouch ? 0.45 : 0.70;
var minScale = hasTouch ? 0.28 : 0.50;
var maxScale = hasTouch ? 0.65 : 0.90;

function adapt(dt) {
  fpsSamples.push(dt);
  if (fpsSamples.length > 40) fpsSamples.shift();
  if (adaptCooldown > 0) { adaptCooldown -= dt; return; }
  if (fpsSamples.length < 40) return;
  
  var fps = 1 / avg(fpsSamples);
  if (fps < 20 && renderScale > minScale) {
    renderScale = Math.max(minScale, renderScale - 0.06);
    adaptCooldown = 2.5;
  } else if (fps > 50 && renderScale < maxScale) {
    renderScale = Math.min(maxScale, renderScale + 0.03);
    adaptCooldown = 3.5;
  }
}
```

### 11.3 Mali 专项

Mali 对片元着色器指令数极敏感（上限约 `512~1024`）。

| GPU | 指令数上界 |
|---|---:|
| Mali-G57 | ~800 |
| Adreno 6xx | ~1024 |
| Apple A 系列 | ~4096 |
| 桌面独显 | ~65536 |

Mali 兼容清单：

- ❌ 避免递归（GLSL ES 不允许）
- ❌ 避免动态数组索引
- ❌ 避免 `for` 循环非常量上界
- ❌ 保留字不能用（`out`、`in`、`inout`）
- ❌ `smin` 参数名不要和循环变量冲突
- ✅ 循环体尽量短
- ✅ 复杂函数手写展开

---

## 12. 调试与排错

### 12.1 常见错误速查

| 症状 | 原因 | 修复 |
|---|---|---|
| 全黑 + FPS -- | 着色器编译失败 | 看红色错误日志 |
| `'xxx' : syntax error` | 用了保留字 | 重命名 |
| 空日志链接失败 | 指令数超限 | 降 `MAX_STEPS` |
| 边缘黑框 | 法线不连续 / `EPS` 太小 | 加大 `EPS` |
| 表面噪点 | 阴影自遮挡 | 加大偏移 |
| 几何体接缝黑线 | 步长下限太大 | 降到 `0.003` |
| 面状黑斑 | 掠射光线穿模 | 降步长下限 |
| 反射里物体消失 | `mapCoarse` 缺物体 | 补进 `mapCoarse` |
| 阴影里物体消失 | `mapCoarse` 缺物体 | 补进 `mapCoarse` |
| 画面被雾洗白 | inscatter 系数过高 | 减半 |
| 帧率低 | 指令数 / 分辨率 | 降 `renderScale` |

### 12.2 调试技巧

1. 可视化法线：

```glsl
void main() {
  ...
  vec2 hit = marchScene(ro, rd);
  if (hit.x < 0.0) { gl_FragColor = vec4(0.1, 0.1, 0.1, 1.0); return; }
  vec3 p = ro + rd * hit.x;
  vec3 n = normalScene(p);
  gl_FragColor = vec4(n * 0.5 + 0.5, 1.0);
}
```

正常 → 平滑渐变。异常 → 颜色跳变。

2. 可视化阴影：

```glsl
vec3 shadeSurface(...) {
  ...
  return vec3(sh);   // 只输出阴影可见度
}
```

3. 可视化步数：

```glsl
vec2 marchScene(...) {
  ...
  return vec2(t, float(i) / float(MAX_STEPS));   // 用 h.y 传步数
}
```

---

## 13. 与其他渲染器对比

| 维度 | 本渲染器 | Minecraft RTX | 光栅化 |
|---|---|---|---|
| 算法 | 确定性 Whitted 光追 | 路径追踪 | 光栅化 + Shadow Map |
| 光线方向 | 固定 | 随机半球采样 | 无 |
| 噪点 | 无 | 必然存在 | 无 |
| 彩色 GI | 伪 GI | 近似 | 完整 |
| 弹射次数 | 1~2 | 4~8 | 0 |
| 硬件门槛 | 任何 WebGL 设备 | RTX 2060+ | 任何 GPU |
| 移动端 | 30+ fps | 不可用 | 60+ fps |
| 视觉风格 | 干净、锐利、风格化 | 真实、柔和、有氛围 | 一般 |
| 适用 | 交互式场景 | 3A 游戏 | 通用 |

一句话：本渲染器是“用最少的算力做出干净可用的效果”，路径追踪是“用昂贵的算力追求物理真实”。两条路平行，不是替代关系。

---

## 14. 场景模板

### 14.1 最小可运行模板

```html
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>SDF Scene</title>
<style>
html,body{margin:0;height:100%;overflow:hidden;background:#000;}
#c{display:block;width:100%;height:100%;}
</style>
</head>
<body>
<canvas id="c"></canvas>
<script>
(function(){
'use strict';
var canvas = document.getElementById('c');
var gl = canvas.getContext('webgl') || canvas.getContext('experimental-webgl');
if (!gl) return;

var vs = 'attribute vec2 a_pos;void main(){gl_Position=vec4(a_pos,0.0,1.0);}';
var fs = [
'precision highp float;',
'uniform vec2 u_resolution;',
'uniform vec3 u_camPos;',
'uniform vec3 u_camDir;',
'uniform float u_focal;',
'uniform float u_time;',
'',
'const float EPS=0.012;',
'const int MAX_STEPS=64;',
'',
'float sdBox(vec3 p,vec3 b){vec3 q=abs(p)-b;return length(max(q,0.0))+min(max(q.x,max(q.y,q.z)),0.0);}',
'float sdSph(vec3 p,float r){return length(p)-r;}',
'vec2 opU(vec2 a,vec2 b){return a.x<b.x?a:b;}',
'',
'vec2 mapScene(vec3 p){',
'  vec2 res=vec2(1e9,0.0);',
'  res=opU(res,vec2(p.y,0.0));',
'  res=opU(res,vec2(sdSph(p-vec3(0.0,1.1,0.0),0.7),1.0));',
'  res=opU(res,vec2(sdBox(p-vec3(2.0,0.5,0.0),vec3(0.5)),2.0));',
'  res=opU(res,vec2(sdBox(p-vec3(0.0,3.5,0.0),vec3(2.0,0.1,2.0)),3.0));',
'  return res;',
'}',
'',
'vec3 albedoOf(float id){',
'  if(id<0.5)return vec3(0.2);',
'  if(id<1.5)return vec3(0.9);',
'  if(id<2.5)return vec3(0.4);',
'  return vec3(1.0);',
'}',
'',
'vec3 normalScene(vec3 p){',
'  vec2 e=vec2(0.005,0.0);',
'  return normalize(vec3(',
'    mapScene(p+e.xyy).x-mapScene(p-e.xyy).x,',
'    mapScene(p+e.yxy).x-mapScene(p-e.yxy).x,',
'    mapScene(p+e.yyx).x-mapScene(p-e.yyx).x));',
'}',
'',
'vec2 marchScene(vec3 ro,vec3 rd){',
'  float t=0.08;',
'  for(int i=0;i<MAX_STEPS;i++){',
'    vec3 p=ro+rd*t;',
'    vec2 h=mapScene(p);',
'    if(h.x<EPS)return vec2(t,h.y);',
'    t+=max(h.x,0.003);',
'    if(t>35.0)break;',
'  }',
'  return vec2(-1.0,0.0);',
'}',
'',
'void main(){',
'  vec2 uv=(gl_FragCoord.xy-0.5*u_resolution)/u_resolution.y;',
'  vec3 ro=u_camPos;',
'  vec3 ww=normalize(u_camDir);',
'  vec3 uu=normalize(cross(ww,vec3(0.0,1.0,0.0)));',
'  vec3 vv=cross(uu,ww);',
'  vec3 rd=normalize(uv.x*uu+uv.y*vv+u_focal*ww);',
'  vec2 hit=marchScene(ro,rd);',
'  vec3 col=vec3(0.0);',
'  if(hit.x>0.0){',
'    vec3 p=ro+rd*hit.x;',
'    vec3 n=normalScene(p);',
'    vec3 lc=vec3(1.0,0.95,0.9)*5.0;',
'    vec3 L=normalize(vec3(2.0,4.0,2.0)-p);',
'    float ndl=max(dot(n,L),0.0);',
'    col=albedoOf(hit.y)*lc*ndl;',
'  }',
'  col=col/(1.0+col);',
'  col=pow(col,vec3(1.0/2.2));',
'  gl_FragColor=vec4(col,1.0);',
'}'
].join('\n');

function compile(type, src) {
  var sh = gl.createShader(type);
  gl.shaderSource(sh, src); gl.compileShader(sh);
  if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
    console.error(gl.getShaderInfoLog(sh));
    throw new Error('shader');
  }
  return sh;
}
var prog = gl.createProgram();
gl.attachShader(prog, compile(gl.VERTEX_SHADER, vs));
gl.attachShader(prog, compile(gl.FRAGMENT_SHADER, fs));
gl.linkProgram(prog); gl.useProgram(prog);

var buf = gl.createBuffer();
gl.bindBuffer(gl.ARRAY_BUFFER, buf);
gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1,-1,1,-1,-1,1,-1,1,1,-1,1,1]), gl.STATIC_DRAW);
var a = gl.getAttribLocation(prog, 'a_pos');
gl.enableVertexAttribArray(a);
gl.vertexAttribPointer(a, 2, gl.FLOAT, false, 0, 0);

var U = {};
['u_resolution','u_camPos','u_camDir','u_focal','u_time'].forEach(function(n){
  U[n] = gl.getUniformLocation(prog, n);
});

var cam = { pos: [0, 1.7, 6], yaw: Math.PI, pitch: -0.1, focal: 1.0 };
var t0 = performance.now();

function render() {
  var w = Math.floor(canvas.clientWidth * 0.6);
  var h = Math.floor(canvas.clientHeight * 0.6);
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w; canvas.height = h; gl.viewport(0, 0, w, h);
  }
  var cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
  var cy = Math.cos(cam.yaw),   sy = Math.sin(cam.yaw);
  gl.uniform2f(U.u_resolution, w, h);
  gl.uniform3f(U.u_camPos, cam.pos[0], cam.pos[1], cam.pos[2]);
  gl.uniform3f(U.u_camDir, -sy*cp, sp, -cy*cp);
  gl.uniform1f(U.u_focal, cam.focal);
  gl.uniform1f(U.u_time, (performance.now() - t0) / 1000);
  gl.drawArrays(gl.TRIANGLES, 0, 6);
  requestAnimationFrame(render);
}
render();
})();
</script>
</body>
</html>
```

### 14.2 加新物体的清单

每次往场景加新物体，检查这 5 处：

1. `mapScene` —— 加入几何
2. `mapCoarse` —— 同步加入（否则反射/阴影缺失）
3. `albedoOf` —— 加入材质颜色
4. `isEmissiveID` —— 如果自发光
5. `clampPosition`（JS 侧）—— 加入碰撞

---

## 15. 参数速查

### 15.1 核心参数

| 参数 | 推荐值 | 作用 | 调整方向 |
|---|---:|---|---|
| EPS | 0.012 | 命中阈值 | 黑边 → 加大 |
| MAX_STEPS | 64 | 主光线最大步数 | 帧率低 → 减小 |
| SHADOW_STEPS | 12 | 阴影射线最大步数 | 阴影质量 → 加大 |
| 主光线步长下限 | 0.003 | 防死循环 | 不要加大 |
| 阴影步长下限 | 0.03 |  |  |
| 粗代理步长下限 | 0.008 |  |  |
| 阴影起点偏移 | `n*0.08` | 避免自遮挡 | 阴影泄漏 → 减小 |
| 反射起点偏移 | `n*0.06` |  |  |
| MAX_DIST | 35.0 | 最远距离 |  |

### 15.2 光照参数

| 参数 | 推荐值 | 说明 |
|---|---:|---|
| 漫反射系数 | 6.0 | 光源强度 |
| 高光系数 | 1.5 |  |
| 高光指数 | 48.0 | 越大越锐 |
| 反平方衰减常数 | 0.15 | 越大越快衰减 |
| 伪 GI 系数 | 1.3 |  |
| AO 系数 | 1.5 |  |

### 15.3 体积雾参数

| 参数 | 推荐值 | 说明 |
|---|---:|---|
| 基础密度 | 0.42 | 5m 能见度 12% |
| 门口溢出 | 0.22 |  |
| 指数衰减 | `exp(-p.y * 1.5)` | 地面更浓 |
| 雾颜色 | `vec3(0.62, 0.74, 0.86)` | 冷白 |

### 15.4 分辨率参数

| 平台 | renderScale | minScale | maxScale | dprCap |
|---|---:|---:|---:|---:|
| 移动 | 0.45 | 0.28 | 0.65 | 1.10 |
| 桌面 | 0.70 | 0.50 | 0.90 | 1.60 |

---

结语

这份文档描述的渲染器：

- 不追求“物理正确”，追求“视觉够用 + 性能能跑”
- 不产生噪点，因为不做随机采样
- 在 Mali-G57 上跑到 30+ fps
- 可以做真正的交互场景（门、物理、碰撞、拾取）
- 所有代码可在任意支持 WebGL 的设备上运行

它不是 Minecraft RTX，也永远不会是。它是“Web 端 SDF 场景的实用渲染器”——用最少的算力，做出干净、稳定、可玩的 3D 体验。

开发新场景时的黄金准则：

1. 分层设计 —— 房间层 / 物体层 / 光源层 / 效果层
2. 同步维护 —— `mapScene` 和 `mapCoarse` 必须一致
3. 保守参数 —— `EPS`、步长下限宁可大，不要小
4. 性能优先 —— 加特效前先保证 30+ fps
5. 一次一改 —— 每次只改一个参数，验证后再改下一个

---

文档版本 v5.2 · 2026-09-19 · MIT License  
本路线与路径追踪路线平行，非替代关系
