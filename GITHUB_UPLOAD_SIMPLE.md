# 🚀 GitHub 上传 - 简化版操作指南

## 第一步：决定如何处理 piper_sdk 和 piper_ros

### 💡 我的建议（最简单）

**选择 B：直接包含源码**

为什么？
1. ✅ 最简单，别人克隆后直接能用
2. ✅ 不需要学习 Git 子模块
3. ✅ 版本稳定，不会突然变化
4. ✅ 适合新手和项目复现

---

## 第二步：执行以下命令（复制粘贴即可）

### 1. 清理 Git 信息

```bash
cd /home/robot/button/V4.0/project2

# 移除 piper_sdk 和 piper_ros 的 Git 信息
echo "正在清理 piper_sdk 的 Git 信息..."
rm -rf piper_sdk/.git
rm -f piper_sdk/.gitignore

echo "正在清理 piper_ros 的 Git 信息..."
rm -rf piper_ros/.git
rm -f piper_ros/.gitignore

echo "✓ 清理完成！"
```

### 2. 移除已暂存的 piper_sdk

```bash
# 先移除之前添加的
git rm --cached -r piper_sdk 2>/dev/null || echo "piper_sdk 未暂存"

echo "✓ 已移除旧的暂存"
```

### 3. 重新添加所有文件

```bash
# 添加 piper_sdk 和 piper_ros（现在是普通目录了）
git add piper_sdk/
git add piper_ros/

echo "✓ 已添加 piper_sdk 和 piper_ros"
```

### 4. 检查状态

```bash
git status
```

**应该看到**：
- ✅ 很多 "新文件" 在 piper_sdk/ 和 piper_ros/src/
- ❌ 没有 "submodule" 字样
- ❌ 没有 piper_ros/build/ 和 piper_ros/devel/

---

## 第三步：提交到本地仓库

```bash
cd /home/robot/button/V4.0/project2

# 提交所有更改
git commit -m "Initial commit: Piper button control system

Features:
- Vision-based button detection using RealSense D435i and YOLO
- Support for 4 button types: toggle/plugin/push/knob
- MoveIt integration for smooth trajectory planning
- Interactive button selection interface
- Complete deployment documentation

Technical Stack:
- ROS Noetic
- MoveIt
- OpenCV + PyTorch
- Intel RealSense SDK
- Piper SDK (included)

Deployment:
- Fully portable with relative paths
- Detailed setup guide in DEPLOYMENT_GUIDE.md
- Ready for easy reproduction on other machines"
```

---

## 第四步：创建 GitHub 仓库（网页操作）

### 1. 打开 GitHub

浏览器访问：https://github.com

### 2. 创建新仓库

1. 点击右上角 **+** → **New repository**
2. 填写信息：
   - **Repository name**: `piper-button-control`（或你喜欢的名字）
   - **Description**: `Vision-based robotic button control system for Piper arm`
   - 选择 **Public**（公开）或 **Private**（私有）
   - ⚠️ **不要勾选**任何初始化选项（README, .gitignore, License）

3. 点击 **Create repository**

### 3. 复制仓库地址

创建后会显示类似：
```
https://github.com/你的用户名/piper-button-control.git
```

**把这个地址记下来！**

---

## 第五步：连接远程仓库并推送

### 1. 关联远程仓库

```bash
cd /home/robot/button/V4.0/project2

# 替换为你的仓库地址
git remote add origin https://github.com/你的用户名/piper-button-control.git

# 验证
git remote -v
```

应该看到：
```
origin  https://github.com/你的用户名/piper-button-control.git (fetch)
origin  https://github.com/你的用户名/piper-button-control.git (push)
```

### 2. 推送到 GitHub

```bash
git push -u origin master
```

### 3. 输入认证信息

**会要求输入**：
```
Username for 'https://github.com': 你的GitHub用户名
Password for 'https://你的用户名@github.com': 你的Token（不是密码！）
```

⚠️ **重要**：GitHub 不再接受密码，需要使用 **Personal Access Token**

---

## 🔑 如何获取 GitHub Token（第一次需要）

### 快速步骤：

1. GitHub 右上角头像 → **Settings**
2. 左侧最下方 → **Developer settings**
3. **Personal access tokens** → **Tokens (classic)**
4. **Generate new token (classic)**
5. 填写：
   - **Note**: `piper-project` （备注）
   - **Expiration**: `90 days` 或 `No expiration`
   - **Select scopes**: 勾选 **repo** （所有 repo 权限）
6. **Generate token**
7. **立即复制 Token**（只显示一次！保存好）

### 使用 Token：

```bash
# 第一次推送时
git push -u origin master

# 输入用户名后，密码处粘贴 Token
Username: 你的用户名
Password: ghp_xxxxxxxxxxxxxxxxxxxx（你的Token）
```

### 保存认证（可选，避免每次输入）：

```bash
# 配置凭证缓存（15分钟）
git config --global credential.helper cache

# 或永久保存（不太安全，但方便）
git config --global credential.helper store
```

---

## 第六步：验证上传成功

### 1. 查看 GitHub 网页

打开你的仓库：`https://github.com/你的用户名/piper-button-control`

应该能看到：
- ✅ README.md 自动显示在首页
- ✅ 所有文件和目录
- ✅ piper_sdk/ 和 piper_ros/src/
- ❌ 没有 build/, devel/, __pycache__/

### 2. 测试克隆（可选）

```bash
cd /tmp
git clone https://github.com/你的用户名/piper-button-control.git
cd piper-button-control
ls -la
```

---

## 🎉 完成！

### 分享你的项目

把这个链接给别人：
```
https://github.com/你的用户名/piper-button-control
```

### 别人如何使用

1. **克隆项目**：
   ```bash
   git clone https://github.com/你的用户名/piper-button-control.git
   cd piper-button-control
   ```

2. **查看部署指南**：
   ```bash
   cat DEPLOYMENT_GUIDE.md
   ```

3. **按步骤安装**：
   - 安装系统依赖（ROS, MoveIt, RealSense）
   - 安装 piper_sdk
   - 编译 piper_ros
   - 创建 conda 环境
   - 运行系统

---

## 📝 后续修改如何上传

### 修改文件后：

```bash
cd /home/robot/button/V4.0/project2

# 查看修改
git status

# 添加修改
git add 文件名
# 或添加所有修改
git add .

# 提交
git commit -m "描述你的修改"

# 推送
git push
```

### 示例：

```bash
# 修改了 button_actions.py
git add button_actions.py
git commit -m "Fix: 修复按钮检测的边界问题"
git push

# 更新了文档
git add README.md DEPLOYMENT_GUIDE.md
git commit -m "Docs: 更新部署文档，添加故障排除章节"
git push
```

---

## ❓ 常见问题快速解答

### Q: 推送时说仓库太大怎么办？

**A**: 检查是否包含了不该上传的文件：
```bash
# 查看大文件
find . -type f -size +50M

# 如果是 .pt 模型文件
git rm --cached *.pt
git commit -m "Remove large model files"
git push
```

### Q: 推送很慢或卡住？

**A**: 
1. 检查网络连接
2. 尝试使用代理或 VPN
3. 或者压缩后手动上传

### Q: 忘记 Token 了？

**A**: 重新生成一个：
1. GitHub Settings → Developer settings → Personal access tokens
2. 点击旧的 Token → Delete
3. Generate new token (classic)
4. 复制新的 Token

### Q: 想添加 License（开源协议）？

**A**: 
1. GitHub 仓库页面 → **Add file** → **Create new file**
2. 文件名：`LICENSE`
3. 右边出现 **Choose a license template**
4. 选择 **MIT License**（最常用）
5. Commit new file

---

## ✅ 完整操作清单

- [ ] 清理 piper_sdk 和 piper_ros 的 .git 文件
- [ ] 移除不需要的大文件（模型、缓存等）
- [ ] 重新添加所有文件
- [ ] 提交到本地仓库（git commit）
- [ ] 在 GitHub 创建新仓库
- [ ] 获取 Personal Access Token
- [ ] 关联远程仓库（git remote add）
- [ ] 推送到 GitHub（git push）
- [ ] 验证上传成功
- [ ] 测试克隆和使用

---

**准备好了吗？开始吧！** 

按照上面的命令一步一步执行即可。有任何问题随时问我！🚀
