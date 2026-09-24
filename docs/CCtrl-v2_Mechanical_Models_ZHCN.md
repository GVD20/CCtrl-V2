# CCtrl-V2 机械模型

[ZHCN](CCtrl-v2_Mechanical_Models_ZHCN.md) | [EN](CCtrl-v2_Mechanical_Models_EN.md)

本页汇总 CCtrl-V2 的机械装配、改进替换件及外观图。

## 主要文件与目录

| 文件 | 内容 |
|---|---|
| [CCtrl-v2-assembly.step](../hardware/mechanical/cad/CCtrl-v2-assembly.step) | 完整装配模型 |
| [CCtrl-v2-replacement-parts.step](../hardware/mechanical/cad/CCtrl-v2-replacement-parts.step) | 主模型中部分零件的改进版本，包含内走线与稳定性设计 |
| [CCtrl-v2-cover.png](../hardware/mechanical/renders/CCtrl-v2-cover.png) | 透明背景渲染主图 |
| [机械资料说明](../hardware/mechanical/README.md) | 模型文件关系 |

## 模型关系

完整装配 STEP 描述六轴控制器的整体结构。替换件 STEP 包含用于更新完整装配中对应零件的局部改进模型；替换关系由两份 STEP 的零件几何与装配位置确定。封面图展示外观，透明通道保留在 PNG 中。

控制器的关节 A1–A6 与六路 Encoder 数据对应，前两段连杆等长。接收端按目标机械臂的实际连杆尺寸建立运动学映射；控制器固件输出关节计数。

## 相关文档

- [用户手册](CCtrl-v2_User_Manual_ZHCN.md)
- [硬件工程资料](CCtrl-v2_Hardware_Engineering_ZHCN.md)
