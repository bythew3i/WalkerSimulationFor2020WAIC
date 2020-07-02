> ! This is private repo for **The Walking Dead** team.

- [The Walking Dead Docs](./docs.md)
- [Sync](#sync)
- [Task Status](#task-status)
- [Rules](#rules)



## Sync
```bash
# Add UBTECH-Walker as upstream
git remote add upstream https://github.com/UBTECH-Walker/WalkerSimulationFor2020WAIC.git


# Push to the Team private repo
git push origin master

# Pull/Fetch from Team private repo
git pull origin master
# or
git fetch origin master



# Pull/Fetch from UBTECH-Walker (Only for sync purpose)
git pull upstream master
# or
git fetch upstream master

```

## Task Status

### (总计100分) 运动控制模式 Control Mode
- [x] ~~（10分）任务1：开电灯~~ 
    ```
    rosservice call /walker/sence "{scene_name: SwitchLight, nav: false, vision: false}"
    ```
- [x] ~~（20分）任务2：拿饮料罐~~ 
    ```
    rosservice call /walker/sence "{scene_name: GraspCup, nav: false, vision: false}"
    ```
- [x] ~~（30分）任务6：推平板车~~
    ```
    rosservice call /walker/sence "{scene_name: PushCart, nav: false, vision: false}"
    ```
- [x] ~~（40分）任务10：开冰箱~~
    ```
    rosservice call /walker/sence "{scene_name: OpenFridge, nav: false, vision: false}"
    ```

### （总计60分） 运动控制+视觉模式 Vision Mode
> 完成一项40分；两项50分；三项60分
- [x] ~~任务3：拿饮料罐~~
    ```
    rosservice call /walker/sence "{scene_name: GraspCup, nav: false, vision: true}"
    ```
- [ ] 任务7：推平板车
    ```
    rosservice call /walker/sence "{scene_name: PushCart, nav: false, vision: true}"
    ```
- [x] ~~任务11：开冰箱~~
    ```
    rosservice call /walker/sence "{scene_name: OpenFridge, nav: false, vision: true}"
    ```


### （总计75分）运动控制+视觉+导航模式 Navigation Mode
> 计分表述不明 
- [ ] 任务4：拿饮料罐
    ```
    rosservice call /walker/sence "{scene_name: GraspCup, nav: true, vision: false}"
    ```
- [ ] 任务5：拿饮料罐
    ```
    rosservice call /walker/sence "{scene_name: GraspCup, nav: true, vision: false}"
    ```
- [ ] 任务8：推平板车
    ```
    rosservice call /walker/sence "{scene_name: PushCart, nav: true, vision: false}"
    ```
- [ ] 任务9：推平板车
    ```
    rosservice call /walker/sence "{scene_name: PushCart, nav: true, vision: false}"
    ```
- [ ] 任务12：开冰箱
    ```
    rosservice call /walker/sence "{scene_name: OpenFridge, nav: true, vision: false}"
    ```
- [ ] 任务13：开冰箱
    ```
    rosservice call /walker/sence "{scene_name: OpenFridge, nav: true, vision: false}"
    ```


### （总计120分）挑战任务
> 搬箱子：
> 1. （20分）将箱子提起离开桌面
> 2. （40分） 直线行走至红色引导线
> 
> 上楼梯：
> -  0 ~ 10s：60分
> - 10 ~ 20s：50分
> - 20 ~ 30s：40分
> - 30 ~ 40s：30分
> - 40 ~ 50s：20分
> - \>= 60s：10分
> 

- [x] ~~（60分）任务14：搬箱子~~
    ```
    rosservice call /walker/sence "{scene_name: CarryBox, nav: false, vision: false}"
    ```
- [x] ~~（60分）任务15：上楼梯~~
    ```
    rosservice call /walker/sence "{scene_name: Upstairs, nav: false, vision: false}"
    ```

## ChangeLog
- 2020-06-20 #2 新增前向运动学Service, 支持左右手
- 2020-06-20 新增反向运动学Service，仅支持左手




## Rules

赛事概述 

WAIC黑客马拉松作为世界人工智能大会期间的重磅赛事和特色环节，将于2020年7月8日-11日在线上举办。ROBO GENIUS是优必选科技重磅打造的机器人及AI教育创新成长平台，聚集各领域专家、老师、学生及生态伙伴，提供贯穿K12到高校的各类人工智能及机器人赛事。优必选科技ROBO GENIUS作为本次大赛的联合承办方，与机器之心联合举办了走进未来·Walker大型仿人服务机器人仿真挑战赛（以下简称“Walker仿真挑战赛”）。  
  
赛事主题

让人形机器人进入家庭，成为家庭重要的一员，是优必选科技从未改变过的目标。Walker是优必选科技研发的中国第一款可商业化落地的大型仿人服务机器人，与波士顿动力的Atlas、本田的Asimo等共同入选“全球5大人形机器人”。

Walker具备36个高性能伺服关节以及力觉、视觉、听觉等全方位的感知系统，集合了优必选科技多年来在人工智能及机器人领域的尖端科技，可以实现平稳快速的行走和灵活精准的操作，具备了在常用家庭场景和办公场景的自由活动和服务的能力，开始真正走入人们的生活。

本届比赛基于仿真平台开放Walker模型及相关数据，邀请更多高校、科研机构以及顶级开发者们参加Walker仿真挑战赛，研发完成15项不同难度的任务，共同推动大型仿人服务机器人的落地应用和研发。
