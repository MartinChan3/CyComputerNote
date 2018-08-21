# QSS

## 样式规则
样式表包含一个选择器和一个声明。选择器用来指定具体使用样式表的对象，而声明则指定哪些属性因此而改变；例如：
```
QPushButton { color: red }
```
上述的样式规则指定了PushButton和它的子类统统使用红色的前景色；

Tip：通常情况下，QSS并不对大小写敏感，QSS只对类名、对象名、Qt属性名是大小写敏感的；

前方的选择器内容可以使用逗号对内容进行并集处理，而其设定的样式属性中可以用分号进行分隔；

选择器|例子|解释
------|----|----
通用选择器|*|匹配所有部件
类型选择器|QPushButton|匹配实例QPushButton以及它的子类
属性选择器|QPushButton[flat="false"]|例子是用来匹配非扁平的QPushButton实例。该选择器几乎可以匹配所有的Qt属性（只要支持该属性可以用字符串表示）。除此之外， 也支持特别的类名来指定类名；同理选择器可以用于测试动态属性；除了=，同样可以使用~=来测试是否Qt属性QStringList包含一个给定的QString;
类选择器|.QPushButton|匹配QPushButton的实例化对象，但是不匹配它的子类，等价于*[class~="QPushButton"]
ID选择器|QPushButton#okButton|匹配所有QPushButton类的实例中ID名叫okButton的；
后代选择器|QDialog QPushButton|匹配所有是QDialog后代的QPushButton（包括子对象，孙子对象）
子选择器|QDialog > QPushButton|匹配所有QDialog的直接QPushButton实例

##子控件
对于相对复杂的控件，可以指定其特定的一部分进行操作，例如一个QSpinBox的向上向下箭头，这就是所谓的子控件修改概念：
```
QComboBox::drop-down { image: url(dropdown.png)}
```
QSS虽然形式上接近css3的内容，但在层级语义上有相当不同；
子控件修改的位置概念总是相对于另一个基准元素。这个基准元素一般可以是widget或者其他的子控件；例如,通常QComboBox的向下箭头默认都是放在右侧顶端，而下拉的内容菜单会放在中间；
原始的矩形框可以通过subcontrol-origin属性来进行修改；下面的例子中给出了ComboBox向左缩进20%的情况；
```
QCombox{
	margin-right: 20px;
}
QComboBox::drop-down {
	subcontrol-origin: margin;
}
```
drop-down的对准政策主要由subcontrol-position属性来进行控制；
width和height属性可以用来控制子控件的大小。注意如果对子控件用一张图片设定了，其实隐性的指定了一个子控件的大小；
同时，位置组合允许子控件相较于其原始位置有一定的初始位移(position:relative)。例如我们经常在QComboBox中希望存在一个“按压效果”：
```
QComboBox::down-arror{
	image: url(down_arr.png);
}
QComboBox::down-arror:pressed {
	position: relative;
	top: 1px; left: 1px;
}
```
而(position:absolute)允许子控件的位置相对于参考元素位置有着绝对位移；
一旦位置被确定，它们就可以等同于处理widgets，详情参考[盒模型](https://doc.qt.io/qt-5/stylesheet-customizing.html#box-model)

