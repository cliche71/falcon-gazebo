# 仅供用户以后手动推送

本次本机整理没有执行任何远端修改。新聚合仓库不沿用内层上游HKUST的origin；旧外层没有origin，旧内层远端已在归档保存。先由用户确认自己的目标仓库。

```bash
cd /home/ljz/falcon_gazebo_repair_ws
git status
git log --oneline --decorate -3
git remote -v
# 确认尚无origin后，由用户填写自己的地址：
git remote add origin <用户自己的仓库URL>
git push -u origin main
git push origin v0.1.0-gazebo-aligned v0.2.0-time-consistent v0.3.0-clearance
```

不使用force、mirror或all推送。三个提交是2026-10-27之后按真实当前时间重建的快照导入，不是伪装原始历史日期。原始历史完整归档，原上游版权未统一替换。
