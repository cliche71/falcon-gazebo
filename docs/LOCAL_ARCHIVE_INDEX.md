# 本地归档索引

统一归档根：`/home/ljz/falcon_gazebo_native_backups/consolidation_20261027_164354`。完整原路径、文件清单、模式/执行位、SHA256和符号链接在manifests/；内容按OBJECT_INDEX.json复用既有已核验tar成员，或存在objects/和containers/。不要只复制某个清单而遗漏载荷。原两份基线tar仍在父归档根，索引明确引用。

完整实际保留/删除与空间结果见`/home/ljz/falcon_gazebo_native_backups/consolidation_20261027_164354/CLEANUP_MANIFEST.json`和CONSOLIDATION_RESULT.md；这些文件在验证和实际删除后生成，不在本次三个导入提交之后追加源码提交。

主要映射：native_workspace→旧falcon_gazebo_native_ws；short_workspace→暂停short_segment；old_gazebo_workspace→旧falcon_gazebo_ws；rviz_workspace→旧falcon_ws；main_before→本次整理前实际A及所有A/B报告；observer_before→固定观察工作区及其原始记录。每个manifest记录原绝对路径。旧Git完整元数据（含reflog/stash/untracked工作树）已归档，不并入新的三提交历史。

恢复例：`python3 /home/ljz/falcon_gazebo_native_backups/consolidation_20261027_164354/tools/archive_store.py restore short_workspace /一个空目录/short_restore`。工具校验内容、权限与链接；只恢复到不存在的新目标，不能覆盖现有主线。旧ELF/devel内RPATH和生成脚本可能要求原绝对路径，恢复数据到任意目录不等于可直接运行旧二进制。要运行旧实验，请按原路径关系恢复到隔离环境，或从完整源码重建。

近期成功证据原路径：reports/author_clearance_20261027_112840，原控制68.93秒碰撞，调整130.25/98.58秒原FINISH。大型原始bag/CSV可由main_before恢复；主线只留近期摘要/审阅包与索引。原日志中的历史路径保持原文，不替换成当前路径。
