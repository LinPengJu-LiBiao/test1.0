# test1.0

一个单文件 C++ 小例子（简易计算器），用来练习 Git / UGit 的日常操作：
查看改动、暂存、提交、建分支、合并、丢弃更改、回滚提交。

## 编译运行

用 MSVC（在 “x64 Native Tools 命令提示符” 里执行）：

```bat
cl /nologo /EHsc main.cpp
main.exe
```

用 g++：

```bash
g++ -std=c++17 -Wall main.cpp -o calc && ./calc
```

## 可以拿来练的功能

- 加一个「5) 取余」或者「6) 幂运算」
- 把 `switch` 换成函数指针表
- 增加连续计算（用上一次的结果做下一次的输入）
