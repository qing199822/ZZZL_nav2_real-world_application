# Git 仓库操作指南

## 首次推送（已完成）

```bash
# 1. 安装 GitHub CLI
sudo apt install -y gh

# 2. 登录 GitHub（选 HTTPS，浏览器或 token 认证）
gh auth login

# 3. 初始化仓库
cd ~/ZZZL_nav2_real-world_application
git init
git add -A
git commit -m "init"

# 4. 创建远程仓库并推送
gh repo create ZZZL_nav2_real-world_application --private --source=. --remote=origin --push
```

## Token 登录（推荐）

`gh auth login` 浏览器打不开时，用 token 方式：

1. 打开 https://github.com/settings/tokens
2. Generate new token (classic)，勾选 **repo** 权限
3. 复制 token，运行：
   ```bash
   gh auth login --with-token
   # 粘贴 token
   gh auth setup-git   # 让 git 也用 gh 认证
   ```

## 日常推送

```bash
cd ~/ZZZL_nav2_real-world_application
git add -A
git commit -m "描述修改内容"
git push
```

## 新机器拉取

```bash
git clone https://github.com/qing199822/ZZZL_nav2_real-world_application.git ~/ZZZL_nav2_real-world_application
```

## 忽略规则

`.gitignore` 已排除以下内容不上传：
- `build/` `install/` `log/` — 编译产物
- `*.stl` `*.pcd` — 大型模型文件
- `.composer/` `.claude/` — 工具目录

## 仓库地址

https://github.com/qing199822/ZZZL_nav2_real-world_application
