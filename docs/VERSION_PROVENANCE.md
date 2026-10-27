# 三版重建来源（当前v0.3.0-clearance）

共同官方固定提交312eb4d32c6c7af1a482f94a0a204aa2bb150cca。以下是本次整理按真实当前时间重建的三次导入，不倒签日期。新提交哈希由各标签解析，完整哈希表在归档根COMMITS.json，避免把尚未生成的自身哈希写入其提交内容。

| 标签 | 原始实物 |
| --- | --- |
| v0.1.0-gazebo-aligned | finish_baseline_source_programs.tar.gz，SHA256 5e804627585b680d48c2523a7187cd62877ceda3616506aa47482071f75a5208；1828项恢复核验。FALCON工作树与ba774e8401085d7666a7ed3b1456a83044ed32c4逐文件一致（即使当时内.git HEAD仍显示312eb4d）；接入原44544be97a0aad903c9d088d65d0155d21441283。包含已批准深度A+B、Gazebo对齐和只读诊断，不含时间A。 |
| v0.2.0-time-consistent | 实际A：FALCON bde9cd0028966b6297a5198fe4937e1caf51a826，接入eb5b87916dcb00373c9a25cf8b2ef9313a971044；A_source_programs.tar.gz SHA256 fa0553683d5d75c0d70dfcd958ba32188b7b5e30cb26c06b0fdda1e604e4f8ca，1926项恢复核验。 |
| v0.3.0-clearance | v2代码不变，新增默认profile为占据比例1.5、软避障距离1.0，保留original_a。来自author_clearance_20261027_112840实际三轮记录及程序身份。 |

v1→v2的五项FALCON文件变化与原ba774→bde的A差异逐项核对：删除PUB额外Slow压缩，刷新位置/yaw导数、时长及缓存；包括repair_contract_probe及只读server状态。原lengthenTime、start_time、server求值/切换和原FINISH保留，不含B或short_segment修复。

共同整理只包括相对工作区入口、默认显式official_compatible、构建/数据忽略、profile加载顺序、依赖与归档说明；源内Git元数据、生成cache/二进制与大可选资源不入新Git，已归档保全。v2起另外明确加入本次成功试验所用固定观察支撑的源码材料/环境接入（6b9886a），它是后来只读运行观测的诊断差异，并非把B/短段或新控制数值冒称为A。原数字运算等价校验与实际库哈希在归档原观察及author_clearance报告。

v2→v3的src/FALCON树与规划/控制算法相同；只改变profile默认选择和版本说明。完整导出清单、算法差异核验与fsck在统一归档verification/。
