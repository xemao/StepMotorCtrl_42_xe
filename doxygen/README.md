# 文档生成与发布说明 {#doc_publish}

本目录存放 StepMotorCtrl_42 的 Doxygen 文档配置与手写说明。

| 路径 | 说明 |
| --- | --- |
| `Doxyfile` | Doxygen 配置（`INPUT` = `User` + `Core` + 本目录的 `mainpage.md`、`README.md`） |
| `mainpage.md` | 文档站点首页（中文导读：硬件、执行线、模块地图、单位约定、已知问题） |
| `README.md` | 本文件：文档的生成与发布说明 |
| `docs/*.md` | 手写的框架分析文档（01 分析方法论、02 项目框架地图） |
| `docs/md2html.py` | 把上述 md 转成 html 的小脚本 |
| `docs/*.html`、`html/`、`latex/` | <b>生成物，不进版本库</b>（见仓库根目录 `.gitignore`） |

## 一、本地生成

必须在<b>工程根目录</b>执行（Doxyfile 里 `INPUT` / `OUTPUT_DIRECTORY` 是相对路径，本地与 CI 行为一致）：

```bash
doxygen doxygen/Doxyfile
```

- HTML：`doxygen/html/index.html`
- LaTeX：`doxygen/latex/`（需要 PDF 时见第三节）

## 二、在线发布（GitHub Pages）

`.github/workflows/docs.yml` 在推送到 `master`/`main`（且改动涉及 `User/`、`Core/`、`doxygen/` 配置或文档、工作流本身）时自动执行：

1. 安装 Graphviz（生成包含关系图）；下载 <b>`env.DOXYGEN_VERSION`</b> 指定版本的 Doxygen <b>官方二进制</b>
   （默认 `1.18.0`，与本地开发一致；带 `actions/cache` 缓存，下载或解压失败时自动回退到发行版 doxygen）
2. 生成 HTML 到 `_site/html`（CI 中追加 `GENERATE_LATEX = NO` 以节省构建时间）
3. 上传产物并部署到 GitHub Pages

首次使用需在仓库 <b>Settings → Pages → Source</b> 选择 <b>GitHub Actions</b>（一次性；用 Actions 部署时
不需要 `.nojekyll`，工作流里仍会创建一个作为兜底，以便将来改用"按分支托管"）。

## 三、PDF 与离线包（按需）

- <b>本地 PDF</b>：`doxygen/latex/` 会保留，进入该目录执行 `make.bat`（Windows）或 `make`（Linux/macOS，
  需已安装 LaTeX 发行版）即可得到 `refman.pdf`。<b>CI 里用 `GENERATE_LATEX = NO` 关掉了这一步</b>，
  纯粹是为了省构建时间。
- <b>离线 HTML 包</b>：仓库根目录的 `StepMotorCtrl_42_API_Doc_html.zip` 是本地打好的离线包
  （解压后双击 `index.html`，含中文阅读说明，已加入 `.gitignore`），适合直接发给别人。
- <b>将来要"一个文件"分发</b>：可以再加一个 job——在 CI 里生成 PDF 或 HTML 压缩包，并挂到
  <b>GitHub Release</b> 附件上；这样在线看网站、离线取单文件，两条路都有。
- 其他单文件形态（需要额外工具链，当前未启用）：CHM（`GENERATE_HTMLHELP=YES`，需 HTML Help Workshop 的
  `hhc.exe`）、Qt Help（`GENERATE_QHP=YES`，需 `qhelpgenerator`）、RTF（`GENERATE_RTF=YES`，无需外部工具，
  但图片外链且中文走 GBK 编码，不推荐）。
