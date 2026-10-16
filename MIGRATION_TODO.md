# DGX Spark 课程迁移待办清单

更新时间：2026-10-16（Asia/Shanghai，本轮导航由A接手）

目标：只在 DGX Spark 的新课程容器内运行 Isaac Sim 6.0、ROS 2 Jazzy、Nav2、MoveIt2 和 YOLOE，跑通导航、视觉定位、抓取、提升的完整课程。

本文件是唯一持续更新的进度入口，统一记录待办状态、验证证据、问题和更新历史。迁移设计依据另见 [迁移计划](DGX_SPARK_MIGRATION_PLAN.md)。

规则：`[x]` 表示该项已有验证证据；`[ ]` 表示进行中或待验证。配置文件已写好不等于运行验收通过。每完成一项，更新本清单的勾选、时间、证据及相关问题；不再维护另一份重复的进度报告。

## 全局复盘与恢复路线（2026-10-16，停止继续堆叠导航调参）

- **最新功能检查点（GitHub上传前）：** 按“先跑通”的优先级，真实近桌NavigateToPose重试成功（`reports/course-demo-navigate-retry.json`，终点约37.6mm/yaw0.0051rad）；原相机经MoveIt调整观察姿态后，真实YOLOE DetectObject识别到笔（`reports/course-demo-detect-lookat.json`）。这两项是模块结果，不是从初始场景跑通整套课程。随后在独立demo布局（物体x=1.65）执行接近、闭合、10cm提升，机械臂Action成功但物体没有离桌，报告保持failed（`reports/course-demo-grasp-execute.json`）。已规划收臂并小幅近桌；50%开度的左右夹爪实测均约0.3896rad。下一步优先垂直抓取复验及状态机真实反馈，不再继续精度研究。TODO13–15仍未完成。
- **上传前软件复核：** 公共导航保护14项、readiness非法关节数据5项、隔离域夹爪Action回归7项通过；导航CTest为7/8组，剩余失败为本地course场景SHA与地图来源记录不符。该场景修改来源未完成审查，保留本地、不纳入本次代码提交；不修改断言，不声称全绿。详见 `reports/github-upload-2026-10-16.md`。
- **用户最新优先级覆盖：** 已明确“不要要求太精确，先能跑通再说其它”。原0.55m净距、0.05m跟踪、30mm定位及附加余量作为后续诊断，首次演示保留实际避碰/控制异常停车、真实识别、真实MoveIt执行及物体离桌。停止控制器/通用观察器扩展，优先原课程固定桌边抓取链，再导航整合与精度/重复性。子代理只读审查已完成：`reports/course-scope-review.md`；恢复计划已同步。历史失败不追认通过，原USD和B代码保留。
- **历史停点与课程范围核对（后续进展见本节顶部）：** 用户指定以 `bobac_robotac_src/src/README.md` 核对主线，已安排只读子代理审查，并暂停新的运动。Isaac暂停state2，无导航目标/导航栈/录包；机器人约(2.8375,0.9750)，尚未到站。原README的核心为YOLOE→抓取点→MoveIt→/hand_command→夹爪→提升；步骤文档另有先Nav2到桌边的独立阶段。RPP、ConnectedSmac、world(2.35,.6,π)、物体x1.65与倾斜姿态均为实验候选，不能当成原课程固定要求。保留用户的0.55m/0.05m保护和现有工作，优先收敛固定安全站位下的正向视觉/原Action抓取链，具体审查见 `reports/course-scope-review.md`（完成后以报告结论为准）。TODO13–15未完成。
- **最新长路线的实际结果：** 一致路径策略（有效时复用、10秒或失效重规划）与双时钟里程计保护，在真实AMCL下从(7.114,2.265)绕过北侧隔墙至(2.8375,0.9750)。498秒墙钟试验取消并停稳；轨迹净距下界0.754446m、最大当前路径偏差32.472mm，29条重规划均记录。因采样时刻定位误差首次30.028mm超过内部30mm门禁取消，包含制动定位峰值30.135mm；不是到站成功。普通取消后约2.94mm达到观测低速停稳。完整证据 `reports/world-recovery-consistent-retry-*`，未放松门槛或改写失败。
- **里程计看门狗修复：** 前一次一致路径试验因0.5秒墙钟超时取消，录包显示实际停顿0.533635秒期间仿真时间未推进，下一条odom源时间仅增16.667ms。公共观察器改为源时间150ms、仿真收包间隔150ms及墙钟1.5秒共同约束，允许一个50ms平滑步的DDS时钟话题先后差；暂停、断流、回退、无效/远未来时间仍拒绝。14项容器回归先红后绿，真实498秒复验未再发生该误取消。`world-recovery-odom-watchdog-{red,green}.log`、`world-recovery-consistent-odom-gap.json`。
- **夹爪反馈改造状态：** 保留原Action与/hand_command，新增真实整机/joint_states关节反馈、按仿真时间小步目标、到位稳定/源时间/墙钟超时、互斥及及时取消。隔离域146的真实rclpy Action回归7项（含暂停/回退/非法输入）旧版先失败、新版全通过；包重建安装通过，真实消息中关节名与角度已核对。真实空载第一小目标从0→0.0344rad且Action成功，但探针在下一目标前的时间检查失败，整套空载报告仍为failed，未继续发控制；当前为空夹爪小开度，未接触物体。`reports/gripper-feedback-*`。不能勾选物理夹持/抓取；状态机的恒True/等待成功缺口仍未处理。
- **本轮初次长路线观察器复验：** 新观察器仍未到站，由A取消低效试验；不是保护越界。189条重规划全部经过完整净距及物理起点门禁，最大当前路径偏差12.328mm、定位误差11.917mm、实际轨迹净距下界1.228036m；旧路径偏差156.007mm单独留证，没有再误触发。206.3秒墙钟/130.1秒仿真仅位移0.248m；12秒只读采样95%在转向、仅5%有平移指令，路径首段存在先南后北的栅格折角，Smac平滑迭代设为0。原失败/取消/录包保留在 `reports/world-recovery-staging-retry-*`。官方平滑器独立静态对照净距通过，但其运行在真实起点连接之前；未将该候选固化为最终配置。
- **真实初段连接与重规划对照：** 整顶点邻域连接仍可能在一格内向南再向北。新增真实反例先红后绿，改为在一格对角范围内取线段与边界的交点，逐段碰撞检查、范围外顶点及≤1cm密度保留；目标容器导航重建6.61秒。随后真实长路线134次重规划全部通过、最大跟踪11.470mm，但实测仍77.3%采样在转向，由A取消；不是单靠连接修复就通过。`world-recovery-prefix-*`。以已安装1.3.13官方一致路径树为依据，在独立无Spin实验树中加入有效路径复用，未覆盖旧入口/树；该运行策略尚未作为最终课程入口交付。
- **观察器修复及原失败：** 前一次北绕长目标已取消、停稳；52条重规划全部净距通过，停车时原始路径偏差29.237mm，实际当前已验路径仅15.807mm且无向外增长。只读重算 `world-recovery-staging-reference-analysis.json` 明确观察器错误地继续约束旧路径。公共工具改为跟踪通过完整净距及物理起点门禁的当前发布路径，两次位置对同一路径计算增长率，并保留原始路径偏差、路径索引及源时间快照。12项容器回归先红后绿；本轮重新运行12项通过，真实189次重规划未再误用旧路径。原失败保留，不追认通过。

- **恢复计划2a实测与定位复现：** 七次有界脉冲（含六方向）完成普通停车；合并5684条原生odom，只在首帧建立一次world对齐，2052个源时间比较点的原生位置误差最大0.357mm，map位置最大12.062mm，时间区间速度不确定度最大0.343mm。扫描使用完整倒装旋转；不改变AMCL参数。`world-dynamic-localization-combined-analysis.json`。首次探针因5秒未来TF历史不足中止，失败保留，随后等待真实TF就绪再运行。
- **固定直线复验仍失败，但层级已明确：** MPPI固定0.3m诊断线（不是全局规划）净距门禁退出0、下界0.874853m；实际最大跟踪11.332mm、轨迹净距0.994150m，因定位36.6mm越过30mm门禁取消并停稳。完整录包 `world-recovery-fixed-mppi-bag/`；重算原生odom误差最大0.183mm、AMCL源位姿在1181.0667s误差37.143mm，该修正约5秒后进入TF触发取消。`world-recovery-fixed-mppi-analysis.json`，原失败保留。
- **AMCL最小对照：** 对应Nav2 1.3.13官方OmniMotionModel的alpha1–5均为0.2，重采样后的XY标准差达约0.134/0.147m，与本次近乎精确的原生仿真odom不匹配。独立运行副本将该组系数降至0.001，保留5秒时间容差、更新阈值、地图和公共保护。真实重采样发生后，源时间map误差最大0.220mm；但MPPI固定线跟踪仍失败，预测取消时41.159mm、含制动47.104mm。`world-recovery-amcl-noise-retry-{motion,analysis}.json`。候选只是局部定位改进，不代表长路线或课程通过。
- **公共等待修复：** 真实TF间隔0.657s墙钟、仅0.1s仿真推进使原0.5s墙钟等待误报。保持原采样时间戳，改为0.15s仿真新鲜度与1.5s墙钟同时约束；暂停超时、回退、非有限值拒绝，未用latest TF。容器10项公共保护先红后绿，原失败保留。`world-recovery-capture-wait-{red,green}.log`。
- **RPP固定线通过：** 官方已安装RPP、alpha候选、原地图与仿真时钟平滑器；真实FollowPath Action=4/error=0，最大跟踪1.896mm、轨迹净距下界0.715437m，终点47.351mm/yaw0.014061rad，零速度普通停稳。`world-recovery-rpp-motion.json`。这是人工诊断线，不是全局规划或暂驻站位通过。
- **全局路径最小修复：** 新增仅限一格对角邻域、逐段检查代价图的起点裁剪，连接精确请求终点；保持中段栅格、未知/碰撞拒绝及独立整段0.55m门禁。真实折返/终点/阻挡反例先红后绿，共享overlay重建5.47s通过。10节点active，ConnectedSmac/RPP/角度进度检查/仿真时钟平滑器实际加载；北向短目标真实规划终点误差0、净距0.714418m。
- **全局短目标第一次仍不计通过：** NavigateToPose Action=4、52条重规划均通过、最大跟踪18.000mm、轨迹净距0.715735m，定位误差最大2.445mm；停稳后实际终点50.967mm略超课程50mm门槛，报告保持失败。`world-recovery-global-motion.json`。仅将Nav2内部到达容差收紧50→40mm留出余量，课程50mm门禁不变；已按当前真值重新规划再验。
- **真实全局短目标复验通过：** 当前真值重新规划，整段净距门禁退出0、下界0.724895m；NavigateToPose Action=4/error=0，17条运行重规划全部通过，最大跟踪7.272mm、真实轨迹净距下界0.731096m、终点39.128mm/yaw0.020972rad、定位误差最大5.156mm，输出最高约0.03892m/s与0.08rad/s，正常零输出停稳。`world-recovery-global-retry-{plan,motion}.json`及录包。已固化独立 `params/bobac3_navigation_world_rpp_params.yaml`，默认配置保留；这不是(2.35,.6,π)到站、抓取或全课程验收。
- **回归边界：** 公共保护10项通过，ConnectedSmac端点反例通过，导航CTest仍7/8组；唯一失败为原dirty course来源SHA断言，未改断言、未覆盖该场景。`world-recovery-endpoint-ctest.log`。下一步单拐角/北绕长路线及联合抓取条件，TODO13–15保持未完成。
- **单拐角分层通过：** 0.1m前视在90°弯内切角，35.184mm时预测取消、含制动39.282mm，原报告失败保留。仅将未缩放前视改为0.05m后，真实FollowPath Action=4/error=0，最大跟踪21.759mm、轨迹净距下界0.773312m、实际终点39.601mm/yaw0.023183rad、定位最大6.294mm。`world-recovery-corner-lookahead-motion.json`及录包。独立RPP源参数已同步0.05m；该项仍为人工折线诊断，不冒充全局北绕通过。下一步当前位姿重新规划暂驻目标、通过整段门禁后才执行长路线。
- 用户要求先全局分析，再按高通过概率方案推进。已查对应Nav2版本源码、GitHub #5214/#5579/#6072及作者低加速度调查；复盘见 [失败原因](docs/MIGRATION_FAILURE_ANALYSIS.md)，新的顺序见 [分层恢复计划](docs/superpowers/plans/2026-10-16-global-recovery.md)。不再派C；B源码未修改。
- **底盘模型发现与对照：** 元数据轮径0.035m、30个接触球的径向外缘约0.044m。独立 `scenes/test_scene_bobac_6.0.radius-candidate.usda` 仅改三个半径属性，原始USD、恢复副本、dirty course及资产保留。新X基线稳态增益1.223；候选±X/±Y为0.989–0.995、±yaw为0.967/0.979，六方向均在10%筛选范围。基线与候选起始姿态不同，不能称完全配对的唯一因果证明。证据 `world-global-wheel-geometry.json`、`world-global-radius-candidate.json`及各轴报告。
- **普通停稳边界：** 5/6次通过附加0.005严格诊断；负yaw仍失败，晚期wz范围约[-.0080,.0142]rad/s，约4秒yaw变化.009rad，原失败不改写。六次真实记录满足原StoppedGoalChecker的.015m/s、.02rad/s低速条件；该阈值未改，.55m/.05m及预测停车未放松。不能声称物理加速度认证或严格六项全过。
- **完整课程共同候选：** 原物体x1.4未找到所查安全站位的指定抓取周期；物体在桌内x1.65、站位(2.35,.6,π)、30°/60°倾斜时，真实MoveIt含六桌体碰撞的预接近、下降与5cm提升均完整，零控制消息、临时碰撞体已清理。`world-global-joint-tilt-moveit.json`及输入。实际物体未搬动；优先30°候选，正向视觉、真实接触与携物碰撞尚未验证。
- **短目标对照仍失败：** MPPI出现进度失败后由A取消；RPP触发预测跟踪停车，最大偏差.024779m、约.002921m后观测停稳。两者真实规划分别有约49/21.6mm初段折返，终点量化约58.5/49.6mm，不能称同一固定直线的控制器比较。`world-global-short-grid-confounds.json`；没有长目标或导航成功结论。
- **当前最先处理动态定位：** 单次固定直线FollowPath（明确为人工诊断路径、整段净距已通过）中RPP真实前进约.19m，但因定位对真值误差>.03m取消；启动误差约.0268m、含制动峰值.0449m，最大轨迹偏差.027008m。`world-global-fixed-motion.json`。需把odom、map→odom、倒装雷达旋转及源时间分开验证，不能只凭静态初始对齐继续调控制器。
- **软件闭环缺口已复核：** 夹爪仍按sleep后假定REACHED，状态机位置检查仍有恒True分支；尚未修复或验收，后续先补真实反馈再计抓取成功。
- **退出状态：** 无活动目标，world导航正常停止；Isaac保持新轮径候选、暂停state2、真值有限。快照 `world-global-final-state.json`。本轮未重建最终镜像，原dirty course来源断言仍是已知未通过项，TODO13–15不勾选。下一步先执行新计划2a动态定位前置验证，再处理路径连接和选择课程控制器。

## A接手导航反馈修复（2026-10-16，普通停稳通过、目标仍未通过）

- 用户已明确本轮由A接手导航，无需等待C。保留C已有文件；B视觉文件未修改。
- 安装进程在独立DDS域复现了反馈放大、零请求仍非零以及NaN输出。新增真实平滑器回归先红后绿；最终6项通过，包含正常加速确实使用measured odom，不能用开环斜坡冒充。源码 `src/sim_time_velocity_smoother.cpp`；证据 `reports/world-bounded-feedback-{red,build,green}.log`。
- 修复后普通非零跟踪使用实际odom，另限制绝对输出、连续指令变化及请求区间；零请求/反向先按原减速度降零，避免延迟非零odom反复再生制动指令。异常反馈明确故障停车，不能计普通制动。
- 最终版真实普通制动通过：输入最高.04m/s，输出最高.04且无侧向/角输出，真值峰值.051153m/s；正常零请求后约.30s仿真时间达到持续低速停稳，未紧急停车。记录区间净距下界1.283420m、最大偏差.000246m。`reports/world-bounded-normal-brake.json`。探针两次严格中止分别为首帧初始化顺序及自设.3s墙钟超时；失败文件独立保留，最终按公共1.5s墙钟及.15s仿真源时间检查。
- 来源与公共门禁再次通过：相同恢复USD、13静态碰撞体顶点误差0；真实传感器/TF就绪，10节点active、ConnectedSmac和仿真时钟平滑器加载。`world-bounded-live-sensors.json`、`world-bounded-readiness.json`及导航日志。CTest寻根改为允许显式 `COURSE_PROJECT_ROOT`，全套7/8组通过；原dirty course哈希仍失败并保留，真实恢复副本另有几何验证，不能称全绿。
- 本轮真实目标失败：按当前真值重新初始化world，MPPI试验上限收紧到XY每轴.04m/s、角.08rad/s；平滑器原.08/.08/.15上限和.55m/.05m保护保持。新规划净距下界.635478m；偏差.027017m且向外增长时预测取消，包含制动最大偏差.030891m、轨迹净距下界1.279483m，5条真实重规划均通过整段门禁。取消后约.008091m达到观测停稳，Action状态5。没有输出放大或PhysX非有限故障；仍不是初段跟踪/目标成功。`reports/world-bounded-goal-motion.json`。
- 普通制动的证据边界：只证明本次.04m/s输入的零请求停稳及指令变化率约.15m/s²。原生odom逐采样线加速度导数峰值约.6317m/s²，实测车速高于请求，不能声称物理加速度约束或各方向标定通过。`world-bounded-normal-brake-review.json`。
- 后续只读诊断：当前局部和全局代价图在机器人周围.5m内均为0；局部odom坐标约(.093,-.005)，世界map坐标约(8.093,1.077)，两者不能混作同一坐标。真实安装MPPI库加记录路径的简化运动学回放也出现偏差；对齐权重20→80仅有限改善，**未应用到真实场景**。它不是实际初段优化轨迹回放或物理通过证据。`world-bounded-local-map.json`、`world-bounded-mppi-replay-{baseline,align80}.log`。
- 最终状态：导航已正常停止，Isaac暂停state2，真值有限，约(8.092781,1.076845)，未再发目标。最终6项隔离进程回归、9项公共保护检查通过；CTest仍7/8组通过、dirty course来源断言失败。汇总及输入哈希 `reports/world-bounded-integration-summary.json`；完整迁移13–15继续不勾选。下一步由A继续关联MPPI变换后路径、优化轨迹、反馈及重规划时序，再验证有依据的修复。

## 上轮 ConnectedSmac 容器验收（2026-10-16 下午，失败已保留）

- **构建与启动通过：** 最新导航共享overlay构建7.80s，10节点active；真实ConnectedSmac插件和仿真时钟CLOSED_LOOP平滑器均加载。运行记录 `reports/world-connected-runtime-parameters.json`、`world-connected-navigation.log`。导航包B/C责任范围未由A修改。
- **来源复核通过：** 实际恢复副本SHA9383aec5…与地图记录一致，13个静态碰撞体顶点误差0；机器人资产及源场景哈希匹配。`world-connected-geometry.json`。原dirty course SHA43ae57…保留；CTest源挂载寻根导致2组失败，真实项目路径重跑仅原dirty course哈希用例仍失败，不能称全绿。
- **按真实位置初始化及规划通过：** 真值约(7.99995,1.08199,-.000014)，六轴零位，独立运行参数副本初始化。真实新规划897点，首点与请求起点一致、最大间隔.009885m，整段净距门禁退出0、下界.635165m。`world-connected-plan.json`及clearance文件。未发送导航目标，不计实际初段跟踪通过。
- **普通闭环测试失败：** 输入最高.04m/s，平滑输出已超过配置.08m/s并升至约.293m/s，真值速度.33518m/s；约.05s仿真输出周期已生效但不保证限幅。正常零指令阶段前触发预测保护，未取得完整普通制动记录。随后PhysX非有限值、pose=NaN；已暂停、停导航、正常关闭损坏Isaac，新开干净恢复副本保持停止。无新目标到达结论；旧规划因现场变化不得直接执行。
- **A公共保护补强：** 输出逐轴.08/.08/.15上限和NaN/Inf拒绝、非有限几何与机械臂状态拒绝。9项课程容器回归先红后绿，尚无故障运行复验；不是C闭环修复替代。`world-connected-output-guard-before.log`、`after.log`。
- **下一步依赖C修复：** `reports/world-connected-A-handoff.md`为可转发要求，原始证据及具体任务见 `world-connected-integration-feedback.json`。C需对安装库实测反馈高于请求/绝对上限、零请求及异常值做回归，处理真实输出限幅；A随后重新构建、公共门禁、普通制动、真实规划与目标。保留.55m/.05m及运输包络；TODO13–15仍未完成。

## 最新恢复与独立规划（2026-10-16 下午）

- **当前运行入口改变：** 仅一个 Isaac 实例加载 `scenes/test_scene_bobac_6.0.recovery.usda`，从已提交 `7c66ada` 的课程场景独立导出，SHA256 `9383aec555c3111e13a12e7811f46bae9566ee94a18a3dd4d793022b31c86237`。导航未启动、无导航目标；机器人从该副本初始条件重新播放，旧停稳坐标不再代表当前位姿。以下旧段落保留为历史记录。
- **真实故障与保留：** 本轮开始时 Isaac 为 STOPPED=0；恢复播放后六轴关节变为 NaN，PhysX 报无效刚体变换。发现未提交 `.course.usda` 存在 GUI/刚体状态覆盖和异常大位移，来源与因果尚未确认。暂停、归档到 `.runtime/recovery-20261016/` 并正常关闭旧实例；没有覆盖、撤销或提交该场景改动。记录 `reports/scene-runtime-recovery.json`；失败的只规划前置检查保留，不能计通过。
- **恢复验证通过（有限范围）：** 12秒墙钟观察取得578条有限六轴状态，保持零位；20秒真实传感器与采样时刻TF通过，新公共 readiness 连续健康3秒通过。证据 `scene-recovery-health.json`、`live-sensors-scene-recovery.json`、`readiness-scene-recovery.json`。不等于闭环、复位三次或容器重启验收通过。
- **公共门禁修复：** `course_bringup` readiness 对机械臂 position/velocity/effort 的 NaN/Inf 明确拒绝。容器5项回归先红（4失败）后绿；单包共享overlay构建0.78秒通过；真实健康门禁通过。证据 `readiness-nonfinite-before.log`、`readiness-nonfinite-after.log`、`readiness-nonfinite-build.log`。负向测试在节点内部提供真实消息类型，不向共享话题发布无效值；尚未做全栈故障注入。
- **导航交接新增证据：** `reports/world-low-speed-start-diagnosis.json` 关联真实失败记录哈希，精确起点到首栅格点约59.33mm，连接方向向西南，下一段向西北，控制输出却以正Y/正yaw为主。不能据此单独归因执行器、MPPI或量化；请C分析路径起点/裁剪/评分/原生odom反馈。维持0.55m净距及0.05m跟踪约束，无新修复前不重复发相同目标。
- **机械臂无运动规划：** 使用完整 `maps/world_scene_geometry.json`、六桌碰撞体、真实MoveIt，仅假想目标x=1.70/1.80；两站位接近均完整，但10cm提升分别仅约67%/81%、90%/43%，仍失败。`table-grasp-cycle-x170-valid-input-plan-only.json`、`table-grasp-cycle-x180-plan-only.json` 均记录零控制命令与临时碰撞体清理。实际物体位置未改，未执行近桌或机械臂运动。5cm提升另行评估，不混称10cm成功。
- **桌面支撑筛查：** 桌板x范围[0.7,1.7]，marker宽约18.8mm；x1.70/1.80不满足完整桌面支撑，不能用作物理课程候选，即使5cm规划完整也不计可实施方案。`reports/table-hypothetical-support-review.json`；桌内x1.65的5cm提升另行验证，近桌站位导航与0.55m运输净距尚未联合通过。
- **桌内5cm只规划通过：** 假想目标x1.65、预接近指尖z=.65919645、抓取z=.63919645、提升z=.68919645；reach_outer(2.15,.6,π)与reach_close(2.10,.6,π)均真实预接近规划成功，下降与5cm提升fraction均1.0，零控制命令、临时碰撞体清理成功。证据 `reports/table-grasp-cycle-supported-5cm-plan-only.json`。这仅为候选：实际marker仍x1.4，未验证近桌导航、识别、夹持、携物碰撞与执行；5cm不冒充10cm成功。
- **C新改动尚在开发：** 检测到导航插件、控制反馈分析及world参数的新文件；`reports/parallel-navigation.md`最新节明确“离线修复进行中”。A仅只读检查，尚未编译或运行这些进行中改动；等待C最终交接。C指出原MPPI输入路径缺少报告中加入的精确起点连接段，PathAlign初段可能跳过，且smoother墙钟定时与仿真步长不同；仍不得仅靠离线修复宣称物理控制通过。
- 步骤13–15仍不勾选。B/C专属源文件本轮未修改；未重建最终课程镜像。

## 最新低速 world 验收（2026-10-16）

- 已在目标课程容器构建 C 的最新 world profile，9 项运输控制与 6 项路径检查通过；reset 后 10 个 lifecycle 就绪。实际参数记录见 `reports/world-low-speed-runtime-parameters.log`、`world-low-speed-controller-runtime.yaml`：local 半径 0.64m、空 footprint、每轴 XY 上限 0.08m/s、加速度 0.15m/s²、20Hz、CLOSED_LOOP、有效 `GridBased.smoother.max_iterations=0`。当前镜像基础未重建，本轮使用持久 overlay。
- 真实新规划到 `(2.35,0.6,π)` 的门禁退出0，整段中心净距下界 **0.635688m**。证据：`world-low-speed-plan.json` 和 `.clearance.json`。
- 公共运动工具已替换旧 0.48m 取消阈值，按当前速度、反馈仿真时间区间、保守制动预算及向外偏差速度提前停车；运行时每条 `/plan` 均检查整段净距并记录起点连接。取消后继续采集真实制动。预算采用未认证的 0.10m/s² 制动下限、0.40s 反应时间与测量余量，不能当作普适物理保证。五项算法检查通过。
- **本轮真实目标仍未通过。** 偏差 **0.019537m**、向外速度 **0.045654m/s** 时触发预测跟踪停车，取消并发送有界 20Hz 紧急零 Twist。包含制动的已记录轨迹中心净距下界 **1.437889m**，最大偏差 **0.037636m**；3 条运行重规划均通过净距检查。它们证明本次提前停车保住记录区间的几何门槛，不能计为到达目标。
- 原生 odom 仿真时间记录：最大合平移速度 **0.063385m/s**、角速度 **0.160035rad/s**；原始逐采样速度导数超过参数标称加速度，尚未通过物理加减速验收。触发到连续低速停稳约 **0.50s 仿真时间**，直线移动 **0.015063m**；此为取消加紧急零命令的制动，不能冒充平滑链单独制动测试。
- SDK `GetEntityState` 源时间戳始终0，已核对本地官方 `entity_utils.py`。真值记录明确保留 source0 与请求/接收 `/clock` 区间；odom 保留原生 source stamp，Twist 标注仿真接收时间。未用墙钟导出速度/加速度，也不宣称真值为精确源采样时间。
- 汇总：`reports/world-low-speed-motion-review.json`；全量速度/odom/真值/重规划：`world-low-speed-motion.json`。C 离线工具 `world-low-speed-trace-gate.json` 正确退出1（目标被取消，即使几何通过也不算验收通过）。交接更新至 `reports/world-navigation-integration-feedback.json`。
- 退出记录：导航服务进程均干净退出；本轮 RViz 在 SIGINT 超时后被 SIGTERM，退出 -11，已保留 `world-low-speed-navigation.log`，纳入步骤14重启/退出问题，不能称全栈干净退出。
- **现场：** 导航栈已停止、无活动目标。机器人约在 world `(7.997826,1.119579,0.119910rad)`，下次必须按实际位置初始化或再次 reset，不能盲用初始定位。Isaac、机械臂及视觉节点保留；TODO13–15不勾选。下一步由 C 诊断新低速控制的初段跟踪/反馈和实际加减速，A 应用后重新走定位、规划、净距与低速门禁。

## 已完成

- [x] **01. 盘点机器、课程和迁移范围。** 确认 DGX Spark / ARM64 / GB10，保留原始场景及 Action 接口。本轮只验收 Spark。
- [x] **02. 下载官方 Isaac Sim 6.0 ARM64 镜像并固定摘要。** 镜像摘要见 `Dockerfile`。官方镜像内 `VERSION` 实际为 `6.0.0-rc.59+release.41464.5f2772bc.gl`，保留该版本记录。
- [x] **03. 将完整 Isaac SDK 放到桌面。** 路径：`/home/nvidia/桌面/IsaacSim-6.0.0`；完成记录：`reports/sdk-desktop-setup.log`。
- [x] **04. 执行官方 GPU 兼容性检查。** 当前驱动 `580.82.09`，检查结果 `PASSED`；证据：`reports/compatibility-check.json`、`reports/compatibility-check.log`。GUI 已在第10项实际验证。
- [x] **05. 校验并补齐课程资产。** 完整 ZIP 的 616 个条目通过 CRC 校验；桌面解压内容已复制到项目 `isaacsimassets-dev/`；证据：`reports/assets-source.json`。
- [x] **06. 完成原始 USD 审计及独立迁移草稿。** 使用真实 6.0 SDK 审计；草稿有效的本地资产缺失为 0，停用机器人引用不计入运行依赖。缺失的地面标线贴图采用纯色替代。远程环境资产和图执行已在第11项验证。证据：`reports/scene-audit.json`；草稿：`scenes/test_scene_bobac_6.0.draft.usda`。
- [x] **07. 完成 ROS 课程代码首轮 Jazzy 适配及宿主机编译预检。** 9 个包编译通过；已修改 MoveIt 接口、Nav2 配置、关节话题、仿真时间、视觉输入与启动检查。证据：`reports/host-jazzy-build.json`、`reports/host-jazzy-build.log`。容器内编译已在第09项完成。
- [x] **08. 编写容器、桌面挂载及统一启动配置。** `Dockerfile`、`compose.yaml`、`tools/course.sh` 已生成。SDK 挂载到 `/isaac-sim`；桌面只读挂载到 `/workspace/Desktop`；项目读写挂载到 `/workspace/course`。实际挂载和 GUI 可用性已在第10项验证。

## 进行中与后续顺序

**用户已要求恢复推进。** 步骤 11 场景、图和真实传感器验收及步骤 12 通信/三类控制已通过，当前推进步骤 13；步骤 13–15 保持未完成。GitHub 上传由专属子代理检查并提交明确路径，运行与 TODO 仍由 A 负责。

- [x] **09. 完成课程镜像构建。** ARM64 集成镜像已纳入 B/C 完成修改和固定 CLIP tokenizer；镜像内 9 个包编译通过，pip check 无缺失依赖。实际课程容器内持久工作区再次编译 9 个包通过（24.5 秒）。证据：`reports/container-integration-build.log`、`reports/container-build.json`、`reports/container-jazzy-build.log`。镜像 ID：`sha256:40dfb2bdee1ad5d40a2a4d6cdca02e021cbfd6ccb5fd6fb5610d642df7df1a92`。
- [x] **10. 启动新课程容器并验证环境。** 目标容器挂载与 GPU 可见性通过；GB10 CUDA 运算、torchvision GPU NMS、cv_bridge RGB/深度转换及 YOLOE CUDA 推理通过，CLIP 来源已固定、缓存持久化。Isaac Sim 6.0 和 RViz GUI 窗口已在本机桌面实际打开；9 个 ROS 包在容器持久工作区编译通过，apt/pip 版本已导出。证据：`reports/container-environment.json`、`reports/vision-check.json`、`reports/gui-windows.log`。YOLOE 本项使用合成黑图，只证明环境，不代表识别验收。
- [x] **11. 在真实 6.0 运行时完成 Action Graph 迁移。** 8 个图已重建，TF/JointState 使用新版数组接口，Camera 应用 OmniSensorAPI 和 30 Hz tickRate；迁移副本已在真实 SDK 保存、重开。目标容器中的时钟、里程计、雷达、两路关节状态、RGB、米制深度及 CameraInfo 持续采集通过，采样时刻机械臂到相机 TF 可查询。官方暂停/继续/停止重置/重播通过，重播后独立 20 秒全传感器复验通过。图像订阅采用与实际发布端一致的 RELIABLE，解决小接收缓冲下 BEST_EFFORT 丢包；未改宿主网络参数。原始 USD 哈希保持一致。验收汇总：`reports/scene-runtime-acceptance.json`；详见 `reports/sensor-transport-diagnosis.json`、`reports/live-sensors-after-replay.json`、`reports/simulation-cycle.json`。不代表视觉 Action、导航或物理抓取通过。
- [x] **12. 验证仿真通信与三类控制。** 新副本在真实 SDK 保存重开，完整公共栈通过通信门禁；底盘经 Twist 平滑链转动约 0.0439rad 并复位，机械臂关节和空夹爪开合均有实际反馈。MoveIt 仅规划零命令、非零 Action 轨迹约 1 秒仿真时间执行并按 FK 到位/复位；无有效时序的轨迹明确拒绝。14 个运行节点 use_sim_time=true，domain44/Cyclone 与相机 RELIABLE 端点实测一致。证据：`reports/control-runtime-acceptance.json`。仅验收通信和有界控制，导航目标、视觉正向定位与物理抓取属于步骤13。
- [ ] **13. 跑通一次完整课程。** 完成导航到桌边、视觉定位、机械臂接近、抓取和提升；以物体实际随夹爪离开桌面作为成功证据，记录日志和结果。
- [ ] **14. 完成重复性与故障诊断验收。** 固定初始条件，重置后连续成功 3 次；重启容器后再次成功；验证资产缺失、话题无数据、TF 缺失的明确诊断。无数据负向诊断已预检通过，但本项整体尚未完成。
- [ ] **15. 完成交付及教程。** 提供最终场景、可复现构建配置、经过验证的版本锁定、统一启动命令、更新教程和完整验收记录。

## 当前结论

步骤 09–12 已在目标课程容器取得验证证据，当前进入步骤13。默认 6.0 副本 SHA-256 为 `a859a876e4c1829ffe12b0258771ef2f6f0a095cf3a3a03ec37ec30d25b47686`；此前验收版保留在 `reports/scene-before-drive-accepted.usda`，原始 USD 哈希不变。当前唯一仿真加载独立候选场景 `scenes/test_scene_bobac_6.0.course.usda`，保留 MoveIt、关节过滤、机械臂 Action 及 B 新版视觉。首次真实暂驻导航因局部控制净距不足自动取消并停稳，导航栈已停止；机器人约位于 (6.117, 2.840)，尚未回到初始位置，未启动抓取状态机。默认已验收场景文件保持不变。

底盘旧增益造成正向线/角命令反向响应；通过真实小脉冲及 SDK 刚体滚动雅可比定位，现按实际轮几何生成 `linearGain≈1.5`、`angularGain≈-0.07412258`。从草稿重建的新副本相较此前验收版仅变更这两项，保存重开后持续全传感器和底盘转动/复位通过。详情：`reports/drive-control-diagnosis.json`、`reports/base-control-saved-scene.json`、`reports/live-sensors-drive-migrated.json`。未修改宿主网络、驱动或官方 SDK。

机械臂两个实际缺陷已修复：`plan_only` 曾成功却发2条命令，现返回前不发布；缺失加速度约束曾导致非零路径的全部命令同一时刻发布，现加载六轴软件速度/加速度限制各1.0（rad/s、rad/s²，非硬件额定值），并拒绝没有有效时序的轨迹。实际 MoveIt Action 非零轨迹持续约1秒仿真时间，到位及回程 FK 误差为0；当前姿态 FK 对 Isaac 同采样 TF 位置差约0.094mm。证据：`reports/arm-plan-only-current-before.json`、`reports/arm-plan-only-current-after.json`、`reports/arm-motion-timing-before.json`、`reports/arm-invalid-timing-after.json`、`reports/arm-motion-timing-after.json`、`reports/arm-model-live.json`。

B 的 QoS 交接已安装，40项容器局部检查通过，实际三路相机订阅 RELIABLE/depth5，公共启动加载新版课程参数与持久缓存。真实 Action 有界超时、并发拒绝及约0.05秒取消通过（`reports/vision-action-common-stack.json`）；没有正向定位验收。B 的原采样时刻有限 TF 重试已交接并由 A 安装，容器52项检查及真实 Action/暂停/重播/缺TF验收通过；本次正常运行未直接量测到17–50ms的具体等待样本，不宣称已实测全部延迟分布。A 未改视觉专属文件。

导航9节点 active、sim time及只读门禁通过，尚未发目标。已导出 `reports/navigation-scene-geometry.json` 请 C 核验地图与真实世界：旧图189×120、0.05m，场景墙体布局和初始位姿需要对齐。原场景未放置可抓取笔；现已在独立候选场景布置官方 YCB large marker 并完成两轮落桌/复位验证。尚未验证相机正向定位和物理抓取，不能把缺少目标时的超时当作识别通过。

已知退出问题：一次公共栈关闭中 MoveIt 收到 SIGINT 后未在5秒内退出，launch升级SIGTERM后退出码-11（`reports/course-stack-drive-migrated.log`），其余节点正常退出；之后新栈启动与有时序轨迹验证正常。该退出问题与容器重启复现尚未验收，保留至步骤14。底盘/机械臂/夹爪控制通过不等于物理抓取通过，步骤13–15不勾选。

执行通道：普通沙箱仍在执行前报 bwrap loopback 权限错误；使用有具体范围的沙箱外请求。历史偶发自动审批超时不是 Isaac 崩溃证据。未更改宿主权限策略。本轮源码已安装到容器持久工作区；基础课程镜像ID仍为步骤09记录，最终发布前须按最终源版本重建镜像。

## 已知步骤13新增验证

- **可复位目标：** `tools/prepare_course_target.py` 生成独立 `.course.usda` 候选，在 table2 上布置官方 NVIDIA YCB marker，质量0.02kg、凸包碰撞、摩擦材质；资产及纹理来源/哈希见 `reports/marker-asset-source.json`。真实 SDK 加载后，通过官方服务观测两轮下落均约3.541mm并稳定，停止后回到 authored 初始位置；证据 `reports/course-target-physics.json`。仅证明落桌/复位，不证明纹理外观、识别或物理抓取。
- **候选通信复验：** 20秒全传感器持续采集及采样时刻 TF 通过，公共 readiness 通过：`reports/live-sensors-course-target.json`、`reports/readiness-course-target.json`。真实 RGB 快照 `reports/course-target-initial-camera.png` 显示初始相机尚未看到目标，需导航到桌边并调整观察姿态。
- **手指偏移几何：** 当前腕部姿态计算目标偏移曾造成约24.88mm错误；现按请求/候选腕部旋转计算0.166m偏移。前后真实只规划证据 `reports/arm-offset-before.json`、`reports/arm-offset-after.json`；新场景可达方向回归 `reports/arm-offset-course-target.json` 通过，零控制命令。
- **方向失败处理：** 指定抓取方向无IK解时原来静默采用当前方向并成功，真实只规划复现后改为明确拒绝。`reports/arm-orientation-refusal-before.json` 保留原错误成功证据；`reports/arm-orientation-refusal-after.json` 实测 abort/status6、零命令。修复后 `reports/arm-motion-orientation-after.json` 的小幅真实往返通过并恢复六轴零位。
- **验收约束：** 夹爪现有 Action 仍以等待时间报告完成，状态机中到位检查也有占位逻辑。后续抓取验收必须直接观察关节、末端与目标实际运动，不能仅依据这两个接口的 success。
- **协作：** B 已通过专属报告交接 TF 等待修复，A 安装及真实复验通过；C 的地图/世界对齐与桌边目标专项仍进行中。A 未修改其专属文件，C 正式交接后再编译和集成。当前新增证据不勾选步骤13–15。

## 世界导航与桌边规划：当前门禁

- C 的独立世界地图入口已编译并真实启动，10个lifecycle/readiness通过。C 最新配置/地图/路径检查合计19项在目标容器通过。旧地图和默认旧入口保持。
- 严格三维TF投影：900个真实激光点全部距定位图障碍≤0.10m，p95约0.0347m；AMCL初始定位对仿真真值位置差约0.05mm。雷达为倒置安装（roll≈π），A最初用yaw-only投影误判扫描方向；补查矩阵后纠正检查工具，未改雷达或图。证据 `reports/world-navigation-alignment-plan-3d.json`，早期失败诊断保留并标注局限。
- 首版真实北绕路径中心净距约0.183m，未达到运输包络要求。C修订后global实际robot_radius=.64、local/global inflation=.85、planner tolerance=.05，运行参数见 `reports/world-profile-runtime-parameters.json`。当前Jazzy的Smac2D未声明 `GridBased.smooth_path`；不能把该YAML项当成已经禁用平滑的证据。
- 最新**真实重规划**与串接的 `check_path_clearance.py` 均退出0，整段中心净距下界 **0.6351075m**，未知和图外均阻塞，扣0.45m运输包络后余0.1851075m。`tools/world_navigation_check.py` 已传播净距门禁非零失败；保存不可变plan-input以校验哈希。证据 `reports/world-navigation-replanned.json`、`.clearance.json`、`.plan-input.json`。旧路径负向门禁实际退出1，见 `reports/world-old-path-clearance-negative.json`。
- **首次运动失败，不计通过：** 在以上前提和发送前10节点门禁通过后，目标(2.35,.6,π)被接受；局部控制切角，真实位置到(6.380,2.429)时中心净距降至约0.445m，检查器取消目标并最终发送零Twist。停稳位置约(6.117,2.840)，后续8样本停稳检查通过。证据 `reports/world-navigation-staging-goal.json`、`reports/world-navigation-after-cancel-stop.json`；没有碰撞/接触监测证据，不能宣称全程无碰撞。需要C进一步约束local运输包络和速度/跟踪，不能只改global规划后重复发送。
- 原物体(1.4,.6)的含桌子MoveIt只规划：加入全部6个桌子USD碰撞AABB并核对已进入真实PlanningScene；四站位有限IK搜索均未找到无碰撞垂直解，2.02站位的C数值解由真实FK确认位置，但link_2与table2桌板碰撞。`reports/table-reach-moveit-collision-diagnosis.json`。未发送近桌目标或机械臂命令，临时碰撞体已清理。
- 东缘假想目标只规划：x1.6时2.15/2.10有无碰撞末端方案；x1.65、指尖z=.6627376时2.15/2.10的预接近均可规划。进一步接近下降2cm路径fraction=1，但从抓取位提升10cm只有约0.476/0.667，未通过。`reports/table-grasp-cycle-edge-plan-only.json`；物体位置和USD未实际改动。早期请求四元数默认w未清零的报告已标注不可用于垂直结论，以修正后的报告为准。
- 以上是步骤13的部分验证。真实暂驻导航成功、正向视觉定位、物理抓取/提升及重复性均未完成，13–15不勾选。

## 三个并行会话的协调

| 会话 | 可修改范围 | 当前集成状态 |
| --- | --- | --- |
| A（协调与集成） | 容器、环境、场景/Action Graph、公共启动/readiness、机械臂、最终验收及本 TODO | 步骤 09–12 已通过；驱动及轨迹时序修复完成，推进完整课程 |
| B（视觉） | `bobac_robotac_src/src/yoloe_grasp/`、`tools/vision_check.py` | QoS 更新已接收，容器构建及40项局部检查通过；真实三路 RELIABLE、超时/取消/互斥通过，TF 等待已集成、52项容器检查与真实暂停/重播/缺TF诊断通过；正向定位待验 |
| C（导航） | `bobac_robotac_src/src/bobac3_navigation/` | 真实启动、9节点active、只读门禁已由A通过；地图对齐与真实目标待验 |

只有 A 执行共享镜像构建、容器启停、共享编译、Isaac 启动及集成控制栈。B/C 开发期间，A 不修改其负责文件；跨模块需求通过用户转交或专属报告协调。提交只暂存已检查的明确文件，不切换共享分支、不撤销其他会话修改。

B/C 报告须列出修改文件、话题/Action/TF/QoS/时间接口、依赖、已做验证的范围及待 A 执行的容器命令。所有运行通过结论仍需目标课程容器的真实证据。

执行约定：遵照用户要求，在既有共享工作区和当前分支持续集成，不新建 worktree 或切换分支；本 TODO 同时记录决策和验证历史，不另建重复的进度台账。新增技能中的隔离工作区建议服从此明确协作约定。

## 验证范围与待解决问题

- **编译和启动预检范围：** 宿主机构建结果为 `9 packages finished [16.2s]`。`ros2 launch course_bringup course.launch.py --show-args` 返回 0，只验证参数解析，没有启动课程节点。容器内构建、节点启动和功能验收分别在步骤 09–13 记录。
- **启动诊断预检：** 无仿真数据时的 readiness 负向测试按预期退出 1，并列出缺失话题、时钟和 TF，记录见 `reports/readiness-negative.json`。该报告的 `failed` 是预期诊断结果，不表示已经验证真实仿真就绪。实际故障注入验收属于步骤 14。
- **场景资产解析：** 活跃资产通过真实 Kit 场景组合检查；`OmniPBR.mdl` 和远程默认环境在 Kit 中可解析。裸 USD Python 没有 Kit 远程 resolver，其外部 URL 警告不能代替 Kit 运行结论。停用的 mobot/Kaya 不计入活跃运行依赖；虚线 JPEG 在副本采用纯色替代。
- **图和控制兼容：** PhysX 雷达已实际持续发布，键盘轮速控制图已停用。6.0 原生 TF 节点针对本课程 chassis root 未展开全部 sibling link，已按真实刚体显式提供目标；相机补充官方 OmniSensorAPI。两项修复已通过真实重开播放、采样时刻 TF 及停止重播后的数据复验。
- **视觉版本验证：** PyTorch 2.9.0+cu130、torchvision 0.24.0+cu130、NumPy 1.26.4、OpenCV 4.11.0.86、Ultralytics 8.4.26 已在 GB10 通过 GPU/转换/推理检查；CLIP 固定提交 `7ffa84b3bfa40c42ecc1c77147a855e69cb2dd40`，缓存目录 `.runtime/arm64/vision/weights`。真实视觉 Action 已通过无目标超时、并发拒绝和取消诊断；TF 延迟处理已安装并通过异常诊断，正向定位仍待验。
- **显示和运行隔离：** SDK/桌面/项目真实挂载、X11 授权及 Isaac/RViz 窗口通过。所有课程进程同容器，使用 Compose 私有网络隔离其他 domain 44 参与者；容器内仍统一 Cyclone DDS、ROS_DOMAIN_ID=44。
- **控制验收边界：** 官方 `/set_simulation_state` 已成功播放和停止；底盘小角度及复位、机械臂有时序轨迹和空夹爪开合已通过；尚未发送导航目标或抓取任务。公共 `grasp/place` 保留 C 的 navigation_readiness 门禁。

旧进度报告仅作为历史快照归档于 [reports/archive/PROJECT_PROGRESS_2026-10-14_160655.md](reports/archive/PROJECT_PROGRESS_2026-10-14_160655.md)，不再更新；原始日志和 JSON 验证报告继续保存在 `reports/`。

## GitHub 仓库备份

- [x] 初始化本地 Git 仓库与 `main` 分支，整理源码、容器配置、迁移草稿和精选审计记录；原始首个提交为 `3d6c313`。
- [x] 添加根目录 README、本地资产恢复说明和 578 个本地文件的 SHA-256 清单；校验全部通过。排除 SDK、资产、模型、缓存、凭据和原始运行日志，保留在本机。
- [x] 暂存文件检查通过：初始 115 个文件约 3.28 MiB，Python AST、JSON、Shell 语法及 Compose 解析通过，未发现匹配的凭据模式。
- [x] 在 `freeskyyyyy` 下新建 [公开仓库 Isaac4.5-6.0_ProjectAdaptation](https://github.com/freeskyyyyy/Isaac4.5-6.0_ProjectAdaptation)，完成 `main` 分支上传并确认 GitHub 返回 `visibility=PUBLIC`、`isPrivate=false`。本地 `origin` 已关联远端，`main` 已跟踪 `origin/main`。源码、配置、文档、小型场景和精选审计记录已上传；大型资产、模型、SDK、缓存和凭据留在本机。
- [x] 用户恢复任务后，上传子代理完成环境及 B/C 交付的 39 文件审查、提交与推送；远端 `main` 核验为 `2c72746ecfcf0316b194432e3aa3f95ded69f4e7`。后续步骤11快照已上传为 `976686d973b3584cfe22af483311d8390f93595c`；步骤12快照已上传为 `ac08d515513e8b8d7097115c018948e99d8ca840`（报告 `reports/github-upload-todo12.md`）。后续步骤13的机械臂及目标场景修改尚未上传，不代表完整课程已验收。

## 更新记录

- 2026-10-14 16:05：根据当前报告、构建日志和 Docker 状态建立清单。步骤 01–08 完成；步骤 09 正在进行；步骤 10–15 待完成。
- 2026-10-14 16:10：按用户提出的文档重合问题，将独有验证范围和待解决问题并入本文件，旧进度报告归档；今后只维护本文件中的进度。镜像仍在 ROS 等依赖下载阶段，完成项未新增。
- 2026-10-14 16:18：完成 GitHub 上传的本地整理、首个提交和资源清单校验；远端创建与推送等待设备登录。课程镜像依赖下载继续进行。
- 2026-10-14 16:28：GitHub CLI 登录成功，用户要求将新仓库设为公开，开始创建与推送。
- 2026-10-14 16:30：公开 GitHub 仓库创建成功，`main` 首次推送成功，可见性核对为 `PUBLIC`；更新本清单记录上传完成。课程容器构建仍在依赖下载阶段。
- 2026-10-14 16:59：确认 A/B/C 分工和运行协调规则；B/C 报告尚未到达。场景加载隔离预检通过，真实 6.0 图重建进入连接检查；镜像依赖下载约到第 1570 个包，步骤 09–15 均未完成。
- 2026-10-15 19:28：步骤 09 完成。首版镜像已于前一晚完成；本日集成镜像包含固定 CLIP 和 B/C 正式交接修改，镜像内及实际容器内均编译通过 9 个包。挂载和 GPU 可见性通过，步骤 10 进入真实视觉/GUI 验证；尚未计为传感器或闭环验收。

- 2026-10-15 20:19：步骤 10 环境验收通过。真实播放暴露相机 Schema/机器人 TF 问题，修复副本后复验中；步骤 11–15 保持未勾选。公共任务入口加入导航 readiness，防止只看 Action 存在即开始任务。

- 2026-10-15 20:26：用户要求第 10 项完成后暂停，现已暂停后续推进。最近一次目标传感器复验中机械臂到相机 TF 已连通，RGB 仍仅两帧、深度仍无数据（`reports/readiness-sensor-tf-fix.log`）；步骤 11 未通过。相机逐帧执行修改仅写入 `tools/migrate_scene.py`，尚未运行生成或验证；对应重建命令因用户暂停指令被自动审批拒绝，未执行。未启动导航/MoveIt/抓取控制栈；现有容器、Isaac GUI 和 RViz 保留，未额外停止或重启。恢复推进需用户后续指示。

- 2026-10-15 20:38：用户恢复任务，并要求子代理上传 GitHub。A 继续相机逐帧执行复验并增加可选渲染图诊断；上传子代理独立审查已完成环境与 B/C 交付，暂不暂存 A 正在修改的场景/TODO。

- 2026-10-16 00:45：步骤 11 完成。正式入口使用 RELIABLE 图像订阅后，持续采集、采样 TF、官方暂停/继续/停止重播及重播后复验均通过；原始 USD 哈希一致。目标容器重新编译 9 包通过。步骤 12 进入机械臂验证，plan_only 发布顺序缺陷尚未修复，首个隔离目标 NO_IK。按用户要求先上传阶段快照，再压缩上下文；未发送真实机器人控制命令。

- 2026-10-16 01:23：恢复推进步骤12；修复并在容器复验 plan_only 不发命令，机械臂小幅复位及空夹爪开合反馈通过。安装 B 的三路 RELIABLE 更新，40项局部检查及真实超时/互斥/取消通过。导航9节点 active 与只读门禁通过，未发目标。底盘低速试验发现正向线/角命令产生反向响应，已停止导航与控制节点，保留原场景，在独立候选副本推导几何增益；12–15仍不勾选。

- 2026-10-16 01:49：步骤12完成。驱动增益修复从草稿重建、保存重开后通过真实数据/控制；MoveIt时序缺陷有失败、拒绝和有效执行三组证据，14节点sim time统一。默认场景提升为新哈希，保留旧版及原始USD。B的真实QoS/超时/并发/取消通过，正向定位、地图对齐、物理抓取和重复性仍未验收；记录一次MoveIt退出异常。

- 2026-10-16 09:26：步骤13推进中。官方目标候选已在真实容器完成两轮落桌/复位及持续传感器/公共门禁；机械臂偏移及无解方向静默回退修复有真实前后证据，小幅往返复验通过。仅保留独立机械臂栈，等待 B/C 专属交接；完整导航、正向定位与物理抓取仍未完成。

- 2026-10-16 09:32：B TF 等待版安装成功，源码/安装节点SHA256均为 `99b91125f04aad104769d00aa13222d94bc0d3a5e91bfa3452bc25c12be80e27`；目标容器52项检查通过（`reports/vision-tf-wait-tests.log`）。真实 Action 超时3.013s、取消0.049s及互斥通过（`reports/vision-action-tf-wait.json`）。暂停时1.527s超时、0.051s取消，重播时间回退后旧Goal中止、新Goal重新提交图像通过（`reports/vision-lifecycle-tf-replay.json`）。首次生命周期测试在停止后等待旧Goal结束超时，原因是停止期间/clock冻结、重播才发布回退；保留 `reports/vision-lifecycle-tf-wait.json`，未作为通过证据。显式缺失 frame 的真实负向测试通过、零推理提交、取消0.010s（`reports/vision-action-missing-tf.json`），故障注入后恢复正常视觉参数。正向定位、导航和物理抓取仍未验收，13–15不勾选。

- 2026-10-16 10:14：C world入口及最新运输净距版已集成，真实三维激光重合与新北绕规划/整段净距门禁通过（下界0.6351m）；首次真实暂驻目标因局部切角自动取消并停稳，导航未通过，栈已停。含桌子只规划排除原2.02数值解（link_2碰桌板）；东缘假想方案接近可规划但10cm提升未完整。详见新增“世界导航与桌边规划”门禁节；13–15仍未完成。
