import QtQuick 2.5
import QtQuick.Controls 1.3
import QtQuick.Layouts 1.1
import QtQuick.Window 2.2
import SofaBasics 1.0
import SofaApplication 1.0
import SofaData 1.0
import SofaWidgets 1.0
import ImageFilterModel 1.0
import ImageFilterDisplay 1.0

Item {
    id: root
    implicitWidth: layout.implicitWidth
    implicitHeight: layout.implicitHeight
    enabled: sofaScene ? sofaScene.ready : false

    property var indexImageFilters: 0
    property var imageFilter: null
    property var sofaScene: SofaApplication.sofaScene
    property bool configurable: true
    property var comboList: ListModel {
        id: imageitems
    }

    ImageFilterModel {
        id: imagemodel

        //sofaComponent: sofaScene.componentByType("ImageFilter");
        sofaComponent: sofaScene.componentsByType("ImageFilter").at(indexImageFilters);
    }

    function updateIndex(index)
    {
        imagemodel.sofaComponent = (sofaScene.componentsByType("ImageFilter").at(index));
    }

    Component.onCompleted: {
        var models = sofaScene.componentsByType("ImageFilter")
        comboList.clear()
        for (var i = 0; i < models.size(); ++i)
        {
            console.log(models.at(i).name())
            comboList.append({text: models.at(i).name()})
        }
    }

    ColumnLayout {
        id: layout
        anchors.fill: parent
        spacing: 0

        ImageFilterDisplay {
            id: imageFilter
            anchors.centerIn: parent
            smooth: false
            Layout.fillWidth: true
            Layout.fillHeight: true
            width: 500
            height: 500
            imageFilterModel: imagemodel

            Component.onCompleted: {
                update();
            }

            Connections {
                //target: root.imagemodel.sofaComponent ? root.sofaComponent.sofaScene() : null
                //onStepEnd: imageFilter.update()
            }

            MouseArea {
                anchors.fill: imageFilter
                acceptedButtons: Qt.AllButtons
                propagateComposedEvents: true
                hoverEnabled: true
                property int buttonFlags: 0
                property int modifierFlags: 0

                onPositionChanged: {
                    buttonFlags = 0
                    if (mouse.buttons & Qt.LeftButton)
                        buttonFlags += 1
                    if (mouse.buttons & Qt.RightButton)
                        buttonFlags += 2
                    if (mouse.buttons & Qt.MidButton)
                        buttonFlags += 4

                    modifierFlags = 0
                    if (mouse.modifiers & Qt.ShiftModifier)
                        modifierFlags += 1
                    if (mouse.modifiers & Qt.ControlModifier)
                        modifierFlags += 2
                    if (mouse.modifiers & Qt.AltModifier)
                        modifierFlags += 4

                    imageFilter.sendMouseEvent(0, buttonFlags, modifierFlags, mouse.x, mouse.y)
                }

                onPressed: {
                    buttonFlags = 0
                    if (mouse.buttons & Qt.LeftButton)
                        buttonFlags += 1
                    if (mouse.buttons & Qt.RightButton)
                        buttonFlags += 2
                    if (mouse.buttons & Qt.MidButton)
                        buttonFlags += 4

                    modifierFlags = 0
                    if (mouse.modifiers & Qt.ShiftModifier)
                        modifierFlags += 1
                    if (mouse.modifiers & Qt.ControlModifier)
                        modifierFlags += 2
                    if (mouse.modifiers & Qt.AltModifier)
                        modifierFlags += 4

                    imageFilter.sendMouseEvent(1, buttonFlags, modifierFlags, mouse.x, mouse.y)
                }

                onReleased: {
                    buttonFlags = 0
                    if (mouse.buttons & Qt.LeftButton)
                        buttonFlags += 1
                    if (mouse.buttons & Qt.RightButton)
                        buttonFlags += 2
                    if (mouse.buttons & Qt.MidButton)
                        buttonFlags += 4

                    modifierFlags = 0
                    if (mouse.modifiers & Qt.ShiftModifier)
                        modifierFlags += 1
                    if (mouse.modifiers & Qt.ControlModifier)
                        modifierFlags += 2
                    if (mouse.modifiers & Qt.AltModifier)
                        modifierFlags += 4

                    imageFilter.sendMouseEvent(2, buttonFlags, modifierFlags, mouse.x, mouse.y)
                }
            }
        }

    }

    // ////Tool panel
    Rectangle {
        id: toolPanel
        color: "lightgrey"
        anchors.top: toolPanelSwitch.top
        anchors.bottom: parent.bottom
        anchors.right: parent.right
        anchors.topMargin: -6
        anchors.bottomMargin: 20
        anchors.rightMargin: -radius
        width: 250
        radius: 5
        visible: false
        opacity: 0.9

        // avoid mouse event propagation through the toolpanel to the sofa viewer
        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.AllButtons
            onWheel: wheel.accepted = true
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: toolPanel.radius / 2
            anchors.rightMargin: anchors.margins - toolPanel.anchors.rightMargin
            spacing: 2

            Text {
                Layout.fillWidth: true
                text: "Filter parameters"
                font.bold: true
                color: "darkblue"
            }

            ScrollView {
                Layout.fillWidth: true
                Layout.fillHeight: true

                Flickable {
                    anchors.fill: parent
                    contentHeight: panelColumn.implicitHeight

                    Column {
                        id: panelColumn
                        anchors.fill: parent
                        spacing: 5

                        GroupBox {
                            id: savePanel
                            implicitWidth: parent.width

                            GridLayout {
                                anchors.fill: parent
                                columnSpacing: 2
                                rowSpacing: 2
                                columns: 2

                                RowLayout {
                                    Layout.fillWidth: true

                                    Label {
                                        text: "Index "
                                    }

                                    ComboBox {
                                        id: indexViewerComboBox
                                        Layout.fillWidth: true

                                        property  var fakelist: ListModel {
                                            id: tralalol
                                            ListElement { text: "kapoue1"; }
                                            ListElement { text: "kapoue2"; }
                                            ListElement { text: "kapoue3"; }
                                        }

                                        model: undefined !== root.comboList ? root.comboList : fakelist;

                                        onCurrentIndexChanged : switchImageFilter();

                                        function switchImageFilter() {
                                            console.info("Change to " + indexViewerComboBox.currentIndex);
                                            root.updateIndex(indexViewerComboBox.currentIndex);
                                            //root.imagemodel.update()
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    Image {
        id: toolPanelSwitch
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.topMargin: 26
        anchors.rightMargin: 3
        source: toolPanel.visible ? "qrc:/icon/minus.png" : "qrc:/icon/plus.png"
        width: 12
        height: width
        visible: configurable

        MouseArea {
            anchors.fill: parent
            propagateComposedEvents: true
            onClicked: toolPanel.visible = !toolPanel.visible
        }
    }
}
