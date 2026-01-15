# GitHub 上传指南

## 前置要求

1. 安装 Git：
```bash
sudo apt install git
```

2. 配置 Git（如果还没配置）：
```bash
git config --global user.name "richard_udy"
git config --global user.email "your-email@example.com"  # 替换为你的邮箱
```

## 上传步骤

### 1. 初始化 Git 仓库

```bash
cd /home/zyy/Desktop/sp_vision_25-main
git init
```

### 2. 添加所有文件

```bash
git add .
```

### 3. 提交代码

```bash
git commit -m "Initial commit: Campus Competition Code - USB camera armor detection"
```

### 4. 在 GitHub 上创建仓库

1. 访问 https://github.com/new
2. 仓库名称填写：`Campus-Competition-Code`（注意：GitHub 仓库名不支持空格，会自动转换为连字符）
3. 设置为 Public 或 Private（根据你的需要）
4. **不要**勾选 "Initialize this repository with a README"（因为本地已有）
5. 点击 "Create repository"

### 5. 添加远程仓库并推送

```bash
git remote add origin https://github.com/richard_udy/Campus-Competition-Code.git
git branch -M main
git push -u origin main
```

如果使用 SSH（需要先配置 SSH key）：
```bash
git remote add origin git@github.com:richard_udy/Campus-Competition-Code.git
git branch -M main
git push -u origin main
```

### 6. 验证

访问 https://github.com/richard_udy/Campus-Competition-Code 查看你的代码是否已成功上传。

## 注意事项

- 确保 `.gitignore` 已正确配置，避免上传编译产物
- 如果遇到认证问题，可能需要配置 GitHub Personal Access Token
- 仓库名 `Campus Competition Code` 在 GitHub 中会自动转换为 `Campus-Competition-Code`（URL 友好格式）
