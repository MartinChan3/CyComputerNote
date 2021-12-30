# Design Pattern   

[CSDN](https://blog.csdn.net/liang19890820/article/details/66974516)

## UML关系    
[看懂UML图](https://design-patterns.readthedocs.io/zh_CN/latest/read_uml.html )    
1. 泛化(Generalization)
线型：空心箭头+实线    
含义：即继承关系，指向父类   
案例：小汽车与SUV    

2. 实现(Realize)    
线型：空心箭头+虚线
含义：即继承抽象类/接口类，指向接口
案例：小汽车、自行车与SUV    

3. 聚合(Aggregation)    
线型：空心菱形+实线    
含义：弱组合关系，菱形在被组成侧    
案例：学生和班级     

4. 组合(Composition)    
线型：实心菱形+实线    
含义：强组合关系（强组合体现在若整体消失，则部分不复存在），菱形在被组成侧    
案例：部门与公司    

5. 依赖关系(Dependency)    
线型：普通箭头+虚线
含义：描述一个对象运行时会使用到另一个对象，强调临时性关系，通常在运行时产生，并且随着运行时变化可能发生依赖关系变化，从对象指向被依赖的对象     
案例：学生与自行车     

    
