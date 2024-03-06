# XKPG包 的模板工程

## 作业流程
1. Fork本仓库, 更改 `命名空间` 与 `项目名称`
2. 更改 package.xml
3. 更改 `hex-pkg-init.py`
4. 更新 `xpkg-rosdep-meta`, 向其 `data.yml` 添加包名称
5. 如果有任何 SDK 需要本包, 更新 SDK 的 meta 文件
6. 全局搜索 todo 检查有无多余的没做的事情
7. !!删除本文件!!
    
## 注意事项

1. 需要提供 `hex-pkg-init.py`
2. Fork 完之后, 删除本文件, 改成对本包的介绍
