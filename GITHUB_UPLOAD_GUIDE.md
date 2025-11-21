# GitHub 上传完整教程 🚀

## 📋 目录
1. [准备阶段](#准备阶段)
2. [清理不需要的文件](#清理不需要的文件)
3. [创建 GitHub 仓库](#创建github仓库)
4. [上传到 GitHub](#上传到github)
5. [验证和测试](#验证和测试)
6. [常见问题](#常见问题)

---

## 准备阶段

### ✅ 你已经完成的
- ✅ Git 仓库已初始化
- ✅ 相对路径已配置
- ✅ .gitignore 已创建
- ✅ 部署文档已完成

### 📝 需要做的事情

#### 1. 清理不需要的文件

**需要移除的文件（已经暂存但不应该提交）**：

```bash
# 在 project2 目录下执行
cd /home/robot/button/V4.0/project2

# 1️⃣ 移除 __pycache__ 缓存文件
git rm -r --cached __pycache__/
git rm -r --cached utils/__pycache__/

# 2️⃣ 移除 trajectory 运行时文件
git rm -r --cached trajectory/

# 3️⃣ 移除大型模型文件（可选，建议移除）
git rm --cached yolo11n.pt
git rm --cached yolo_button.pt

# 4️⃣ 移除图片文件（可选，如果不需要的话）
# git rm --cached bus.jpg button1.jpg button2.jpg
```

#### 2. 添加修改后的文件

```bash
# 添加所有修改过的文件
git add .gitignore
git add DEPLOYMENT_GUIDE.md
git add PATH_MIGRATION_SUMMARY.md
git add PATH_MODIFICATION_COMPLETE.md

# 添加修改过的脚本
git add *.py
git add *.sh

# 添加 piper_ros 和 piper_sdk
git add piper_ros/
git add piper_sdk/
```

#### 3. 检查状态

```bash
git status
```

**期望看到的结果**：
- ✅ 只有需要的文件在暂存区
- ❌ 没有 __pycache__/
- ❌ 没有 trajectory/
- ❌ 没有 .pt 模型文件（可选）

---

## 创建 GitHub 仓库

### 方式 1：使用 GitHub 网页（推荐新手）

#### 步骤 1：登录 GitHub
1. 打开浏览器，访问 [https://github.com](https://github.com)
2. 登录你的账号（如果没有账号，先注册一个）

#### 步骤 2：创建新仓库
1. 点击右上角的 **+** 号
2. 选择 **New repository**（新仓库）
3. 填写信息：
   - **Repository name**（仓库名称）：`piper-button-control`（或你喜欢的名字）
   - **Description**（描述）：`Vision-based button operation system for Piper robotic arm`
   - **Public/Private**（公开/私有）：
     - ✅ **Public**：任何人都可以看到（推荐，方便别人复现）
     - 🔒 **Private**：只有你能看到
   - ⚠️ **不要勾选**以下选项：
     - ❌ Initialize with README（我们已经有了）
     - ❌ Add .gitignore（我们已经有了）
     - ❌ Choose a license（稍后可以添加）

4. 点击 **Create repository**（创建仓库）

#### 步骤 3：记录仓库地址
创建后，你会看到类似这样的地址：
```
https://github.com/你的用户名/piper-button-control.git
```
**记住这个地址，后面会用到！**

---

## 上传到 GitHub

### 🎯 完整命令流程

#### 1. 回到项目目录
```bash
cd /home/robot/button/V4.0/project2
```

#### 2. 清理不需要的文件
```bash
# 移除缓存文件
git rm -r --cached __pycache__/ 2>/dev/null || true
git rm -r --cached utils/__pycache__/ 2>/dev/null || true

# 移除运行时文件
git rm -r --cached trajectory/ 2>/dev/null || true

# 移除大型模型文件（可选）
git rm --cached yolo11n.pt 2>/dev/null || true
git rm --cached yolo_button.pt 2>/dev/null || true
```

#### 3. 添加所有需要的文件
```bash
# 添加新文档
git add .gitignore
git add DEPLOYMENT_GUIDE.md
git add PATH_MIGRATION_SUMMARY.md
git add PATH_MODIFICATION_COMPLETE.md
git add GITHUB_UPLOAD_GUIDE.md

# 添加修改过的文件
git add -u

# 添加 piper_ros 和 piper_sdk（排除 build/devel）
git add piper_ros/
git add piper_sdk/
```

#### 4. 提交到本地仓库
```bash
git commit -m "Initial commit: Piper button control system

- Vision-based button detection and operation
- MoveIt integration for trajectory planning
- Support for multiple button types (toggle/plugin/push/knob)
- Complete deployment documentation
- Relative path configuration for easy deployment"
```

#### 5. 关联远程仓库
```bash
# 替换为你的 GitHub 仓库地址
git remote add origin https://github.com/你的用户名/piper-button-control.git

# 验证远程仓库
git remote -v
```

#### 6. 推送到 GitHub
```bash
# 首次推送（设置上游分支）
git push -u origin master
```

**可能会遇到的情况**：

##### 情况 A：要求输入用户名和密码
```
Username for 'https://github.com': 输入你的 GitHub 用户名
Password for 'https://你的用户名@github.com': 输入你的密码或 Token
```

⚠️ **注意**：GitHub 现在不接受密码，需要使用 **Personal Access Token**

**如何获取 Token**：
1. 打开 GitHub → 点击右上角头像 → **Settings**
2. 左侧菜单最下方 → **Developer settings**
3. **Personal access tokens** → **Tokens (classic)**
4. **Generate new token** → **Generate new token (classic)**
5. 设置：
   - Note: `piper-project-upload`
   - Expiration: `90 days` 或 `No expiration`
   - 勾选权限：**repo**（所有 repo 相关权限）
6. 点击 **Generate token**
7. **立即复制 Token**（只显示一次！）
8. 在命令行中粘贴 Token（而不是密码）

##### 情况 B：使用 SSH（推荐）
如果你配置了 SSH 密钥，可以使用 SSH 地址：
```bash
git remote set-url origin git@github.com:你的用户名/piper-button-control.git
git push -u origin master
```

---

## 验证和测试

### 1. 检查 GitHub 网页
1. 打开你的仓库页面：`https://github.com/你的用户名/piper-button-control`
2. 应该能看到：
   - ✅ README.md 自动显示
   - ✅ 所有 Python 和 Shell 文件
   - ✅ config/, launch/, utils/ 等目录
   - ✅ piper_ros/src/ 和 piper_sdk/
   - ❌ 没有 __pycache__/
   - ❌ 没有 trajectory/
   - ❌ 没有 piper_ros/build/ 和 piper_ros/devel/

### 2. 在新位置测试克隆
```bash
# 在另一个目录测试
cd /tmp
git clone https://github.com/你的用户名/piper-button-control.git
cd piper-button-control

# 检查文件
ls -la
```

---

## 📝 创建完整的 README.md

在上传前，建议更新 README.md 文件，添加以下内容：

```bash
cd /home/robot/button/V4.0/project2
# 编辑 README.md，添加：
```

**README.md 应该包含**：
1. 项目简介
2. 功能特性
3. 系统要求
4. 快速开始
5. 安装步骤（链接到 DEPLOYMENT_GUIDE.md）
6. 使用说明
7. 项目结构
8. 致谢和许可

---

## 🔧 后续维护

### 修改后如何更新到 GitHub

```bash
# 1. 查看修改
git status

# 2. 添加修改的文件
git add 文件名
# 或添加所有修改
git add .

# 3. 提交
git commit -m "描述你的修改"

# 4. 推送
git push
```

### 创建发布版本（Release）

当项目到达重要节点时：

1. 打开 GitHub 仓库页面
2. 点击 **Releases** → **Create a new release**
3. 填写：
   - **Tag version**: `v1.0.0`
   - **Release title**: `Version 1.0.0 - Initial Release`
   - **Description**: 描述这个版本的功能
4. 上传大型文件（如训练好的 yolo_button.pt）
5. 点击 **Publish release**

---

## ❓ 常见问题

### Q1: 上传速度很慢怎么办？
**答**：piper_ros 和 piper_sdk 包含很多文件，首次上传会比较慢。
- ✅ 使用稳定的网络
- ✅ 如果太慢，可以考虑使用 SSH 方式
- ✅ 或者压缩后作为 Release 上传

### Q2: 模型文件太大无法上传？
**答**：GitHub 单个文件限制 100MB。
- 方案 1：不上传模型文件，在 README 中提供下载链接
- 方案 2：使用 Git LFS（大文件存储）
- 方案 3：上传到 Release（可以上传大文件）

```bash
# 使用 Git LFS
git lfs install
git lfs track "*.pt"
git add .gitattributes
git add yolo_button.pt
git commit -m "Add YOLO model with LFS"
git push
```

### Q3: 忘记移除某些文件怎么办？
**答**：可以在提交后移除：
```bash
# 从 Git 中移除但保留本地文件
git rm --cached 文件名
git commit -m "Remove unnecessary file"
git push
```

### Q4: 如何让别人参与开发？
**答**：
1. **公开仓库**：任何人都可以 Fork
2. **添加协作者**：Settings → Collaborators → Add people
3. **使用 Pull Request**：别人提交修改请求，你审核后合并

### Q5: 怎样添加开源协议？
**答**：
1. GitHub 仓库页面 → **Add file** → **Create new file**
2. 文件名输入：`LICENSE`
3. 右侧会出现 **Choose a license template**
4. 推荐：
   - **MIT License**：最宽松，允许商业使用
   - **GPL-3.0**：要求衍生作品也开源
   - **Apache-2.0**：包含专利授权

---

## 📚 推荐的仓库结构

```
piper-button-control/
├── README.md                          # 项目说明（重要！）
├── DEPLOYMENT_GUIDE.md               # 部署指南
├── LICENSE                           # 开源协议
├── requirements                      # Python 依赖
├── .gitignore                        # Git 忽略配置
│
├── *.py                              # Python 脚本
├── *.sh                              # Shell 脚本
│
├── config/                           # 配置文件
├── launch/                           # ROS launch 文件
├── utils/                            # 工具函数
├── md/                               # 文档
│
├── piper_sdk/                        # SDK 源码
│   ├── setup.py
│   └── piper_sdk/
│
└── piper_ros/                        # ROS 包
    └── src/                          # 只包含源码
```

---

## ✅ 检查清单

上传前请确认：

- [ ] 已清理 __pycache__/
- [ ] 已清理 trajectory/
- [ ] 已处理大型模型文件
- [ ] piper_ros/build/ 和 piper_ros/devel/ 不在仓库中
- [ ] README.md 内容完整
- [ ] DEPLOYMENT_GUIDE.md 已更新
- [ ] .gitignore 配置正确
- [ ] 所有脚本使用相对路径
- [ ] 提交信息清晰明确

上传后请验证：

- [ ] GitHub 页面显示正常
- [ ] README.md 正确渲染
- [ ] 文件结构完整
- [ ] 可以成功克隆
- [ ] 文档链接正常

---

## 🎉 完成！

恭喜！你的项目已经成功上传到 GitHub！

**分享给别人**：
```
https://github.com/你的用户名/piper-button-control
```

**别人如何使用**：
1. 查看 README.md 了解项目
2. 查看 DEPLOYMENT_GUIDE.md 按步骤部署
3. 遇到问题可以提 Issue

---

**最后更新**：2025-11-21  
**作者**：AI Assistant
