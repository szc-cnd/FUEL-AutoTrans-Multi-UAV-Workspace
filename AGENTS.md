# CAV0 项目协作规则

## 本地优先工作流

- Windows 仓库 `D:\Study\codex project\UAV\FUEL-AutoTrans-Multi-UAV-Workspace-cav0` 是 CAV0 代码的唯一修改源。
- 后续代码、YAML、launch、脚本和文档修改必须先在 Windows 本地完成，不得直接在 CAV0 的 `/home/asus/match_ws` 中开发。
- 完成修改后，先在本地运行可用的静态检查和测试，再使用中文提交信息提交到 `vehicle/cav0` 并推送 GitHub。
- CAV0 只负责拉取已提交代码、执行 ROS/catkin 编译、台架检查和实机验证。
- CAV0 验证产生的新日志、bag、点云、图片、视频、`build/`、`devel/` 和 `Testing/` 默认不提交 Git。
- 如果发现 CAV0 工作树存在未提交源码修改，必须先将差异迁回 Windows 本地并核对；不得直接覆盖、丢弃或强制重置。

标准顺序：

```text
Windows 本地修改
  -> 本地静态检查
  -> 中文 Git 提交
  -> 推送 origin/vehicle/cav0
  -> CAV0 快进拉取
  -> CAV0 统一 catkin_make
  -> 台架或实机验证
```

## Git 安全

- 不使用 `git reset --hard`、强制推送或重写远程历史。
- 不提交密码、令牌、SSH 私钥或 GitHub 凭据。
- 不把实验数据和构建产物混入源码提交。
- CAV0 上无法快进拉取时，先停止同步并检查本地与远程差异。
