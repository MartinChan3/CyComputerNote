# 敏捷开发    

## 敏捷需求实践
敏捷需求阶段的要务，就在于制定Product Backlog。Product Backlog既可以包含既有的需求，也可以包含新增的需求（前提：不影响迭代）。该列表最后按照市场价值排序。同时包含产品需求和技术需求。     

需求遵循MECE原则(Mutually Exclusive and Collectively Exhaustive):条目间相互独立+穷举    
[![7ybZcT.png](https://s4.ax1x.com/2022/01/20/7ybZcT.png)](https://imgtu.com/i/7ybZcT)    

用户故事拆分：角色、功能、价值
> 作为[XXX用户]，我要[YYY功能],从而[解决什么问题/原因]        

需求分析7步走：     
[![76Al9K.png](https://s4.ax1x.com/2022/01/20/76Al9K.png)](https://imgtu.com/i/76Al9K)     
除了产品愿景、商业模式画布外
1) 用户角色：寻找产品客户
2) 用户路程：定义产品特征和用户故事
3) 故事拼图：对用户故事进行优先级排序
4) 初始化PB：产生Product Backlog
5) 初始化发布计划：产生初始化的发布计划

内容发展路径：User Personas → User Jounery → User Story → Story Map → Product Backlog → Sprint Backlog → Burning Map        

Step1 用户角色分析：构建用户画像
Step2 用户旅程：1) 与用户访谈；2) 建立流程图；3) 划分story。
注意是从流程图中划分story,从业务中流程功能点对应到story    

[![76gGHf.png](https://s4.ax1x.com/2022/01/20/76gGHf.png)](https://imgtu.com/i/76gGHf)    

获取各个基本的User Story后，需要进行需求拆分，可以按照以下原则进行划分： 
> 按照简单复杂拆分——例如先实现接口，在实现内部内容？    
> 按照操作边界拆分——例如正常管理系统的SCRUD的操作，即把具体的需求可以分成独立不相关的小操作          
> 按不同业务规则切分——先完成业务规则的一个子集，再后续通过新故事添加其他规定     
> 按不同界面切分——如果用户故事有复杂的界面，是否可以有一个简单的版本可以先完成？     
> 延迟性能优化——能否先满足工作，再后续满足性能需求？   
> 按不同类型的数据切分

切分后的几个准则：
> 各个用户故事大小差不多大、且每个Sprint约为6-10个用户故事；    
> 每个故事是否满足INVEST原则；（Independent、Negotiable、Valuable、Estimable、Small、Testable）    
> 是否存在降低或者删除的用户故事？
> 是否有故事可以获得早期价值或者降低风险？    

切分为Stories后可构成故事拼图：在各个用户故事之间找到关联进行分类，并且进行排序。找出之前没有定义出来的产品特性/功能，同时制定初步的发布计划    
[![765esJ.png](https://s4.ax1x.com/2022/01/20/765esJ.png)](https://imgtu.com/i/765esJ)        
[![765DW8.png](https://s4.ax1x.com/2022/01/20/765DW8.png)](https://imgtu.com/i/765DW8)       

故事地图组织后，可形成产品待办事项。     
[![76ICOH.png](https://s4.ax1x.com/2022/01/20/76ICOH.png)](https://imgtu.com/i/76ICOH)    

PB之后，即可初始化发布事项。多个sprint组成一个sprint release。         
[![76IQmj.png](https://s4.ax1x.com/2022/01/20/76IQmj.png)](https://imgtu.com/i/76IQmj)    
     
如果需要写新的非功能需求Story    
