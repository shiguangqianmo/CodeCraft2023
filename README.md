# CodeCraft2023
2023华为软件精英挑战赛-策略部分（C++）

## 3.14
VS2015
完整工程

完成内容：
完成Worktop.h、Worktop.cpp、Robot.h、Robot.cpp、main.cpp文件编写
Worktop.h中包含Worktop类的定义（成员变量、构造函数、参数更新函数）
Worktop.cpp为对应函数实现

Robot.h中包含Robot类的定义（成员变量、构造函数、参数更新函数）
Worktop.cpp为对应函数实现

main.cpp
完成基础框架
定义了一些全局变量
完成初始化函数init()和读帧数据函数read_frame()编写  

## 3.15 
源代码文件

将Worktop和Robot类中的private成员变量全都改为public
Robot类中新增一个bind成员（绑定的工作台的ID）

细化main()的框架


## 3.20
完成策略部分

策略：  
根据value来进行任务的分配，当有机器人空闲时就为其分配一个调度任务 assign：（源工作台，目标工作台），

任务分配流程：

1. 根据工序关系生成所有可能的任务对 （不可靠）
2. 过滤出合法的任务对

    a. 原料台有产品  
    b. 目标台可以接收该产品  
    c. 其他机器人没有占用该原料台  
    d. 其他机器人没有往同一目标台送一样的产品
3. 计算出value最高的合法任务对，并分配给机器人

    value来源：  
    a. 原料台和目标台上所有物品的利润，正数  
    b. 原料台产品以最快速度运输到目标台的价值，正数  
    c. 机器人到原料台的损失（距离*价值系数），负数  

机器人任务执行流程：

1. 机器人绑定原料台
2. 确保在原料台附近，并且原料台上有产品后，buy，并绑定目标台
3. 前往目标台，确保在目标台附近，且目标台能接收手上的产品，sell，解除绑定，清除当前任务

other：
由于合法性判断要求原料台有产品，那么在最初的1s内，将没有合法的任务对，机器人空闲等待1s后才能开始运转。
所以对于初始状态，进行一个特殊first分配，后续机器人空闲了再按照上面的流程走  

## 3.21  

更新内容：

1. 利润计算时，加上潜在产品的利润（一是由于这种工作台更有可能发生堵塞，二是生成任务清单也考虑了潜在产品）
2. 合法性检测不再严格，原料台上有潜在产品，即可成员源目标台
3. 令value_89=0，（7，8）任务依旧是足够有吸引力的任务
4. 构建任务对时，允许构建所有产品到9的任务链，可以解决4号地图的问题（其他地图上好像没有9）
5. 由于合法性检测时，允许有潜在产品的原料台称为源目标台，所以不再需要进行初始分配
6. 机器人到原料台cost，按照d和2vt进行分类计算


新的问题：
（7，8）任务吸引力还是过大，1.机器人会从很远的地方过来，2.机器人会在一个很早的时间就绕着7等待

系数难以权衡：机器人到原料台的cost小一点，工作台上产品价值影响会偏大，
机器人到原料台的cost大一点，机器人趋向保守地等待原料台的产品


新的想法：

1. 
在合法性检测的时候允许有潜在产品的原料台称为源工作台，
但是为机器人分配任务的时候，得判断如果机器人过去这段时间内能生产出来，这个任务才会被该机器人考虑

2. 机器人a往一个目标台送完货，会不管目标台上的东西，重新找一个任务
这种情况很有可能是，另外一个机器人b提前绑定了该工作台作为原料台，所以机器人a无法提取该工作台上的产品
解决：机器人a送完货，如果发现这个目标工作台作为其他机器人b的原料台，那么就抢占机器人b的任务，机器人b空闲，重新分配

## 3.22  

更新内容：

1. is_legal函数限制，不允许将其他机器人的目标工作台作为自己的源工作台。
2. read_frame时更新场上资源，没有4、5、6、7其中一项资源时，该资源将更优先得到生产，实现方式：  
   
	a. `get_assign_list（）`函数更新较多，如果场上没有4、5、6、7资源，（x，4/5/6/7）这类任务将作为优先任务，push到队列头部  
    b. 如果有4、5、6、7资源，（x，4/5/6/7）这类任务将作为备选任务，push到队列尾部  
  	c. 注意，如果目标工作台为8、9，（x，8/9）也将作为优先任务  
    d. 机器人在选择任务对时，将先从优先任务中选择，如果能够选择，则不考虑备选任务对，如果不能选择，则从备选任务对选择  

3. 合法性检测中：不允许两个机器人目标台相同且送的货是同一种，-> 不合理：如果两个机器人目标台为8/9应该允许  
4. 工作台上的原材料价值设置为1，机器人到源目标台的距离有所区分 
5. 为了使（4，7）、...、（6，7）比（1，4）、（2，4）、...、（3，6）更优先，7号台上无产品时，设置7的价值为2*3400（两个3号产品的价值）  

优先任务和备选任务讨论：

 - 有优先任务，无备选任务：4、5、6、7资源都没有，机器人选全局最优
 - 有优先任务，有备选任务：4、5、6、7中部分资源没有，机器人选局部最优，但能使资源均衡
 - 无优先任务，有备选任务：4、5、6、7资源都有，机器人选全局最优
 - 无优先任务，无备选任务：所有工作台都堵塞，机器人空闲（这种情况估计工作台比较少）


问题：机器人还是有过早等待的问题


## 3.23 
完成第二版策略：  

抛弃了第一版的任务对的形式，改用任务链的形式（初始时使用dfs生成所有任务链），目的是使机器人调度过程流畅。  
  
当有机器人空闲时，为机器人分配value最高的任务链（value来源于两部分：机器人到源工作台+工作台之间）。  

每一帧需要读取机器人与工作台状态，根据状态的不同，可行任务链可能是一些子链。为了让4、5、6资源尽量均衡，同样使用了均衡策略，用优先队列分别维护一个优先任务链和一个备选任务链，优先任务链为empty时，取备选任务链队首元素。

## 3.24 
第二版策略debug与优化