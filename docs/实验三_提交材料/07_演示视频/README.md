# 演示视频说明

## 仓库 `videos/` 里现有的两段（**属于实验二，不要删**）

| 文件 | 日期 | 内容 |
|---|---|---|
| `videos/sim_grasp_5of5.mp4` | 2026-09-09 | 实验二 定点抓取 · 仿真 5 连抓（README 内嵌播放） |
| `videos/real_grasp_5of5.mp4` | 2026-09-09 | 实验二 定点抓取 · **真机** 5 连抓 |

配套还有 `logs/real_machine/grasp_real_20260909_*.csv` 共 5 份轨迹日志，全部 SUCCESS。
这两段和这批 CSV 是实验二已验收的证据，README 用 `<video src=...>` 直接引用，删除会让实验二失去演示与实机记录。

## 实验三需要补的录屏

`~/classify_logs/` 下另有 4 个 mp4（`demo_full.mp4`、`demo_full_compat.mp4`、`audience_raw.mp4`、`smoke3.mp4`，9/19–9/21），
都是**旧几何**（r=0.185、瓶 φ65、高壁料盒）下录的过程片段，只能当过程材料，不代表 9/22 最终状态。

按当前配置重录（一条命令起场景，起来即自动跑完 6 格，约 5~6 分钟）：

```bash
bash ~/exp3_gui_run.sh
```

它会先清残留进程、删旧 `~/classify_logs/task_log.json`、以 `gui:=true` 且不暂停启动，
50 秒后打印 `spawned=7` 与最近状态序列，此时开始录。

录之前把 gzclient 相机拉远到能看全桌面（滚轮缩放、中键拖动），默认视角怼在底盘上。
想要固定机位，world 里有 `audience_cam` 模型可跟拍。

**合格判据**（录完对着 `~/classify_logs/task_log.json` 核，别只凭肉眼）：
末条 `summary` 为 `placed=6 failed=0 skipped=0`，四类失败计数全 0。

新录的文件建议命名 `videos/exp3_classify_6of6.mp4`，并在 README 的实验三章节加一行 `<video>` 引用。

## 实验三真机录屏

**缺**。实验二真机已跑通（见上表），但"识别—分类—分拣"整条实验三链路没上实机；
且真机物体是"鼠标 + 网球"，必须启用 YOLO 并自备权重，仿真的轮廓法对低饱和鼠标找不到框。
