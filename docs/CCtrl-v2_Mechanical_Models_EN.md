# CCtrl-V2 Mechanical Models

[ZHCN](CCtrl-v2_Mechanical_Models_ZHCN.md) | [EN](CCtrl-v2_Mechanical_Models_EN.md)

This page lists the CCtrl-V2 assembly model, improved replacement parts, and cover render.

## Main files and directories

| File | Contents |
|---|---|
| [CCtrl-v2-assembly.step](../hardware/mechanical/cad/CCtrl-v2-assembly.step) | Full assembly model |
| [CCtrl-v2-replacement-parts.step](../hardware/mechanical/cad/CCtrl-v2-replacement-parts.step) | Improved parts from the main model, including internal cable routing and stability changes |
| [CCtrl-v2-cover.png](../hardware/mechanical/renders/CCtrl-v2-cover.png) | Transparent-background cover render |
| [Mechanical assets guide](../hardware/mechanical/README.md) | Relationship between the model files |

## Model relationship

The full-assembly STEP describes the six-axis controller mechanism. The replacement-parts STEP contains revised versions of selected assembly parts; correspondence follows their geometry and position in the two STEP files. The cover PNG depicts the exterior and retains an alpha channel.

Joints A1–A6 correspond to the six Encoder channels, and the first two links are equal in length. The receiving robot controller maps joint counts to its own link dimensions and kinematics.

## Related documents

- [User manual](CCtrl-v2_User_Manual_EN.md)
- [Hardware engineering](CCtrl-v2_Hardware_Engineering_EN.md)
