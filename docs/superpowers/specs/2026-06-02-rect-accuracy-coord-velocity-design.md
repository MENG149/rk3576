# 矩形识别增强:坐标系 + 速度预测 + 稳定性

日期:2026-06-02
范围:`cpp_fast_targeting/`,在现有模块上增量增强,不引入新依赖,不破坏现有高帧率主循环。

## 目标

1. 建立统一坐标系:原点在画面中心、y 向下为正、归一化到 [-1,1]。
2. 速度预测:提前量瞄准(lead aiming)+ 丢失时按速度外推(带衰减)。
3. 矩形识别稳定性:角点时序平滑 + 候选评分增强(先验 IOU、凸性/直角约束、先验长宽比)+ 运动自适应 ROI。

## 1. 坐标系(types.hpp,inline 纯函数)

```
dx = px - w/2,  dy = py - h/2          // 像素偏移,y 向下为正
nx = dx / (w/2), ny = dy / (h/2)        // 归一化 [-1,1]
```
角点同样提供归一化版本。无状态,可纯单元测试。

## 2. 速度预测(扩展 RectTracker)

- 复用已有 `vx_/vy_`(像素/秒)。
- 新增配置 `lead_time`(秒,默认 0.05)。`Snapshot()` 输出 `predict_center = (x_+vx_*lead, y_+vy_*lead)`,做边界 clamp。
- 丢失外推:`Predict()` 中速度逐帧衰减 `vx_ *= predict_decay`(默认 0.9),`conf_ *= 0.97`(保留),超过 `max_predict_misses` 帧停止(保留)。
- `TrackSnapshot` 新增:`predict_center`、`vx`、`vy`、`corners[4]`(平滑后角点)。

## 3. 矩形识别(扩展 RectRefiner)

接口新增可选 `RefineHints{ prior_box, prior_aspect, speed }`,保持接口干净。

- **角点平滑**:跨帧状态放在 tracker(已持有跨帧状态)。逐点 EMA `corner = (1-α)*prev + α*new`,`corner_alpha` 默认 0.5;当 source 从预测切回检测、或与上一帧角点位移过大时重置,避免拖影。
- **评分增强**:在原 `area*0.45 + extent*0.30 + center*0.15 + aspect*0.10` 基础上加入与 `prior_box` 的 IOU 项;`aspect` 改为与 `prior_aspect` 的接近度;加凸性 + 近直角约束过滤非矩形四边形。
- **运动自适应**:`speed` 大时临时放大 `roi_expand_ratio`、减弱模糊/形态学核。

## 4. 输出协议(AimWriter,向后兼容尾部追加)

```
$AIM,found,state,cx,cy,dx,dy,nx,ny,px,py,pnx,pny,vx,vy,conf,source
```
- cx,cy 当前中心(像素,原点左上,保持原义)
- dx,dy 中心偏移(像素);nx,ny 归一化
- px,py 预测中心(像素);pnx,pny 预测中心归一化(云台主用)
- vx,vy 速度(像素/秒)
下游只认前缀字段即可,新增字段在尾部。

## 5. 配置新增

`lead_time=0.05`、`predict_decay=0.9`、`corner_alpha=0.5`、`corner_reset_dist=40`、`prior_iou_weight=0.20`、`motion_roi_gain=1.5`、`motion_speed_ref=300`(像素/秒)。均加命令行解析。

## 6. 测试

host 端单元测试(仅依赖数学/OpenCV core,不依赖 RKNN):
- 坐标映射:中心→(0,0),四角→(±1,±1),y 符号。
- tracker 预测:匀速轨迹验证提前量;丢失外推速度衰减。
- refiner 评分:合成图(白底黑矩形)能选中,IOU 项生效。

编译/运行需在 RK3576 上(RKNN+RGA+aarch64);Windows 开发机无法编译整体。
