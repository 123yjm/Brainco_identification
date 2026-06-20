# 计划：将 brainco_identification 工作空间上传到 GitHub

## 背景

工作空间 `/home/ubuntu/Desktop/brainco_identification/` 是一个 C++ 机器人项目（基于 CMake 的机械臂动力学参数辨识），目前还不是 git 仓库，需要上传到 GitHub 用户 `123yjm` 进行版本管理。

**重要提示**：GitHub 从 2021 年 8 月起已禁止使用密码进行 Git 操作。不能直接使用密码，必须使用 SSH 密钥（本机已有）或 Personal Access Token。

## 当前状态

- 项目类型：C++ / CMake 项目，约 3.5 MB
- 不是 git 仓库
- 没有 `.gitignore` 文件 — `build/` 目录（编译产物、CMake 缓存）会被误提交
- SSH 密钥已存在：`~/.ssh/id_ed25519` 和 `~/.ssh/id_rsa`
- GitHub CLI (`gh`) 未安装

## 执行步骤

### 步骤 1：创建 `.gitignore`

在项目根目录创建 `.gitignore`，排除以下内容：
- `build/` — 编译产物和 CMake 中间文件
- `*.o`, `*.a`, `*.so` — 编译生成的目标文件
- `.vscode/`, `.idea/` — IDE 配置文件
- `result/` — 运行输出结果（如果是生成的）

### 步骤 2：确认 SSH 公钥已添加到 GitHub

检查本机 SSH 公钥是否已经关联到 GitHub 账号 `123yjm`：
- 运行 `ssh -T git@github.com` 测试 SSH 连接
- 如果未配置，引导用户访问 https://github.com/settings/keys 添加 `~/.ssh/id_ed25519.pub` 的内容

### 步骤 3：初始化 git 仓库并提交

```bash
cd /home/ubuntu/Desktop/brainco_identification
git init
git add .
git commit -m "Initial commit: 机械臂动力学参数辨识项目"
```

### 步骤 4：在 GitHub 创建仓库并推送

1. 测试 SSH 连接：`ssh -T git@github.com`
2. 在 GitHub 上创建仓库 `brainco_identification`（需确认公开还是私有）
3. 添加远程仓库：`git remote add origin git@github.com:123yjm/brainco_identification.git`
4. 推送代码：`git push -u origin main`

## 需要创建/修改的文件

- **新建**：`/home/ubuntu/Desktop/brainco_identification/.gitignore`

## 验证方式

1. `git log` 显示初始提交
2. `git remote -v` 显示 `git@github.com:123yjm/brainco_identification.git`
3. `git push` 成功
4. 浏览器访问 `https://github.com/123yjm/brainco_identification` 确认文件可见
