# 硬件资料 / Hardware

本目录用于存放 ZOE_TX V1.2 的 PCB 与结构件源文件。建议按下面的结构放置，便于他人复现和二次修改。

```text
hardware/
├─ pcb/
│  ├─ <EDA 源工程>          # KiCad(.kicad_pro/.kicad_sch/.kicad_pcb) / AD(.PrjPcb/.SchDoc/.PcbDoc) / 立创EDA 工程
│  ├─ zoe_tx_v1.2_sch.pdf   # 原理图 PDF(方便没装 EDA 的人查看)
│  └─ fabrication/          # Gerber zip、BOM、坐标文件、贴片图
└─ enclosure/
   ├─ <CAD 源工程>          # Fusion360 .f3d / SolidWorks .SLDPRT/.SLDASM / FreeCAD .FCStd
   ├─ zoe_tx_v1.2_case.step # 中性格式, 任何人可打开
   └─ zoe_tx_v1.2_case.stl  # 直接 3D 打印用
```

## 上传前检查

- **体积**：网页上传单文件 ≤ 25 MB；`git push` 单文件 ≤ 100 MB（> 50 MB 会告警）。
  超过 100 MB 请改用 Git LFS，或作为 Release 附件（单个 ≤ 2 GB）。
- **不要提交**：`build/`、临时导出、压缩包里重复的拷贝、软件缓存。
- **隐私**：Gerber / BOM / 丝印里可能带姓名、电话、地址，上传前请确认是否需要打码。
- **开源硬件许可**：代码用的是 MIT，但硬件资料建议单独声明。
  常见做法是 PCB/外壳采用 **CERN-OHL-S/W** 或 **CC BY-SA 4.0**；若沿用 MIT，请在本文件里明确写出。

## 上传方式

```bash
# 本地整理好后一次性提交(推荐, 能保留目录结构与历史)
git add hardware
git commit -m "docs(hw): add PCB sources, gerbers and enclosure models"
git push
```

也可以直接在 GitHub 网页上 `Add file → Upload files` 拖拽上传，但请先把文件按上面的目录分好类。
