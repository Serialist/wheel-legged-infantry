# wheel-legged-infantry

> Ciallo～(∠・ω< )⌒★

轮腿步兵嵌入式控制代码

## TODO

- [ ] 跳跃
- [ ] 磕上台阶
- [ ] 自瞄
- [ ] 热量限制
- [ ] 功率限制
- [ ] 抗干扰

## Clone

由于一些奇思妙想，导致仓库有两层submodule嵌套

```
wheel-legged-infantry --| dmh7/gb --| robo-lib
                        |
                        | dmh7/ch --| robo-lib
```

克隆时需要使用命令

```shell
git clone --recursive <主仓库URL>
```

如果不幸没加 `--recursive`，可以使用

```shell
git submodule update --init --recursive
```

![Stone Badge](https://stone.professorlee.work/api/stone/Serialist/wheel-legged-infantry)
