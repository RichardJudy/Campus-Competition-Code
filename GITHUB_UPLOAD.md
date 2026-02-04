# 推到 GitHub

装好 git 后，配置下用户名邮箱（没配过的话）：

```bash
git config --global user.name "richard_udy"
git config --global user.email "你的邮箱"
```

本地已经 init 并 commit 过的话，在 GitHub 新建仓库，名字填 `Campus-Competition-Code`（空格会变成横杠），不要勾选初始化 README。然后：

```bash
git remote add origin https://github.com/richard_udy/Campus-Competition-Code.git
git branch -M main
git push -u origin main
```

推送要登录的话用 Personal Access Token。SSH 用 `git@github.com:richard_udy/Campus-Competition-Code.git` 也行。
