# Vendor dependencies

这里集中保存多个模块直接复用的第三方 SDK：`manus_sdk` 和
`xrobotoolkit_sdk`。不要在适配器目录再复制一份。

应用或 ROS 包内部与其源码结构绑定的 vendored 组件仍随所属模块保存；它们不
作为其他模块的公共依赖。
