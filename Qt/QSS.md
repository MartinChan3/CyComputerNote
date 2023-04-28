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

## 伪状态
伪状态经常出现在选择器中来指定widget所处的状态。伪状态(Pseudo-states)出现在选择器的末尾，通常用冒号与指定内容分隔。例如：
```
QPushButton:hover{color:white}
```
类似c当中的否定符，感叹号可以用于非该状态
```
QPushButton:!hover{color:red}
```
伪状态可以构成层级关系，类似于逻辑控制符当中的AND，例如：
```
QCheckBox:hover:checked{color:white}
```
同理，否定状态也可以适用于层级关系的伪状态，例如：
```
QPushButton:hover:!pressed{color:blue;}
```
同样的，类似于OR逻辑的方法通过逗号实现
```
QCheckBox:hover,QCheckBox:checked{color:white;}
```
伪状态还可以与子控制混合使用
```
QComboBox::drop-down:hover{image:url(dropdown_bright);}
```

## 冲突解决
在qss应用过程中，有类似以下情况
```
QPushButton#okButton {color:gray}
QPushButton {color:red}
```
上述情况中因为使用了具体指定okButton来避免了冲突；
相似的，选择器如果具有了伪状态指定，也就相对具有更强的圈定性；
```
QPushButton:hover{color:white}
QPushButton{color:red}
```
改进为以下形式会显得更严格
```
QPushButton:hover{color:white}
QPushButton:enabled{color:red}
```
**qss/css具有第二句优先的特点，即在几条语句同时符合条件的情况下，最下方的语句先起效果**
下方的例子相对更加完善
```
QPushButton:hover:enabled{color:white}
QPushButton:enabled{color:red}
```
同样的问题可能发生在类型选择器当中，例如：
```
QPushButton{color:red}
QAbstractButton{color:gray}
```
这里很明显因为继承关系出现了冲突，虽然编译器可能会尝试通过父子关系来进行推断；但是，对于样式表计算来说，所有的类型选择器都有着相同的处理方式，也就是末位优先原则；换句话说，所有的QAbstractButton都会被设定为grey。如果要正确，则我们必须将样式表重新排序。
qss遵循的顺序完全是按照css2来进行规定的：
> 选择器优先顺序如下：
> 计算选择器当中的ID数量(=a)
> 计算选择器当中伪类和其他属性(=b)
> 计算选择器中的元素名(=c)
> 忽略伪状态（忽略subcontrols）
> 这样构成了一个a-b-c的三位数，决定了顺序
> ```
> *             {}  /* a=0 b=0 c=0 -> specificity =   0 */
> LI            {}  /* a=0 b=0 c=1 -> specificity =   1 */
> UL LI         {}  /* a=0 b=0 c=2 -> specificity =   2 */
> UL OL+LI      {}  /* a=0 b=0 c=3 -> specificity =   3 */
> H1 + *[REL=up]{}  /* a=0 b=1 c=1 -> specificity =  11 */
> UL OL LI.red  {}  /* a=0 b=1 c=3 -> specificity =  13 */
> LI.red.level  {}  /* a=0 b=2 c=1 -> specificity =  21 */
> #x34y         {}  /* a=1 b=0 c=0 -> specificity = 100 */
> ```

## 层级关系
样式表可以应用于QApplication的子对象或者父对象，而任意的widget的样式表都是通过融合父类或者祖父类对象而形成的；
当形成冲突时，widget自身的样式表总是优先于继承的widget内容，而不会考虑冲突内容。相同的，父对象的自身样式表优先级优于祖父对象；
这样带来的一个后果是对一个子对象设定样式表，会导致父类样式表被覆盖；例如：
```
qApp->setStyleSheet("QPushButton{color:white}");
myPushButton->setStyleSheet("*{color:blue}");
```
这种情况下，会迫使QPushButton的所有对象都为蓝色的文本。该情况也等同于：
```
myPushButton->setStyleSheet("color:blue");
```

## 继承
传统css中，当一个item的字体和颜色没有具体指定时，它会从其父类对象中自动推断；**但是，在Qt的样式标系统中，是不会做父类自动推断样式表继承的。** 例如，考虑一个QGroupBox当中的QPushButton：
```
qApp->setStyleSheet("QGroupBox{color:red}");
```
此时，QPushButton没有任何具体设定的颜色，因此，它不会继承父类对象的情况下，它会使用系统默认的颜色；如果想要有继承效果，应该写为：
```
qApp->setStyleSheet("QGroupBox, QGroupBox * { color: red}");
```
相反的，当使用字体设置(setFont)和样式表设置时(setPalette)时，其设置会衍生应用到子类对象中；
如果你希望stylesheet模仿font和palette的衍生效果，你可以使用以下修改方式：
```
QCoreApplication::setAttribute(Qt::AA_UseStyleSheetPropagationInWidgetStyles, true);
```

## C++命名空间内的widgets
类型选择器可以被用于修饰一种特定类型的widgets,例如：
```
class MyPushButton: public QPushButton{
	//...
}
// ...
qApp->setStyleSheet("MyPushButton{background:yellow;}");
```
Qt的样式表使用widget的QObject::className()来决定是否使用类型选择器；因为命名空间内的QWidget返回的通常会是<namespace>::<classname>。这样会导致和subcontrol的命名调用方式形成冲突。为了解决这个问题，使用“--”来替代“::”,例如：
```
namespace ns {
    class MyPushButton : public QPushButton {
        // ...
    }
}

// ...
qApp->setStyleSheet("ns--MyPushButton { background: yellow; }");
```

## 设定QObject属性
从Qt4.3开始，任何可以设计的Q_PROPERTY都可以通过使用qproperty-<property name>的格式，例如：
```
MyLabel { qproperty-pixmap: url(pixmap.png); }
MyGroupBox { qproperty-titleColor: rgb(100, 200, 100); }
QPushButton { qproperty-iconSize: 20px 20px; }
```
如果其属性值为Q_ENUMS，应当直接使用名称调用，而不是数值；

# 使用样式表修饰Qt Widgets

## 盒模型(Box Model)
![box model](http://doc.qt.io/qt-5/images/stylesheet-boxmodel.png)
- 边缘(margin)落在border外界
- 边界(border)落在margin与padding中间；
- 填充(padding)在border内部，在actual contents和border之间；
- 内容(content)是去掉以上三个元素之外的内容；

以上四个为同心的矩形框；其中margin/border-width/padding默认都是0，这种情况下四个矩形完全重合；
可以通过background-image属性来给widget的背景设定图片。默认情况下，background只会绘制border中间的部分；这可以通过background-clip属性来进行修改，同时通过background-repeat和background-origin来控制图片重复和图片原味；
一张背景图片无法修改widget的面积。为了提供一个可以随背景大小变化的控件，必须使用border-image.因为border-image提供了一个可选的背景，无需再重复指定background-image。如果两者都指定时，border-image会覆盖掉background-image。
此外，image属性会覆盖在border-image上层。默认的情况下，图片不会随着widget的尺寸而发生变化，它的排列位置会根据image-position属性而指定。和background-image和border-img不同的是，如果image属性被设定为一张SVG图片，那么图片将会被自动缩放为widget的尺寸；总体的渲染顺序如下：
- 设定整体渲染的裁剪(border-radius)
- 绘制背景(background-image)
- 绘制边框(border-image,border)
- 绘制上层图片(image)

# 子控件
一个widget被默认为子元素堆叠绘制而成。例如QComboBox绘制向下元素，接下来会绘制向下的箭头元素。所以其渲染的习惯如下
-  Render the QComboBox { } rule
-  Render the QComboBox::drop-down { } rule
-  Render the QComboBox::down-arrow { } rule
子控件完全遵循父子原则，子元素使用subcontrol-position和subcontrol-origin属性；
只要位置被确定为，子元素就可以使用盒模型来进行样式设计；

他人qss中内容：
1. 基本上灵活运用的话，需要一定的css基础，而本身算是简单而且有效；
2. 颜色可以rgb(r,g,b)也可以#XXXXXX的形式写出；
3. 