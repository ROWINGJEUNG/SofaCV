import QtQuick 2.0
import QtQuick.Controls 1.3

Row {
    id: root

    property var dataObject: null
    property variant stringList: dataObject.value.toString().split(' ')
    Text {
        text: "dist: "
    }

    TextField {
    id: dist
    readOnly: false
    width: root.width
    text: stringList[0]
    }

    Text {
        text: "imgIdx: "
    }

    TextField {
    id: img
    readOnly: false
    width: root.width
    text: stringList[1]
    }

    Text {
        text: "queryIdx: "
    }

    TextField {
    id: query
    readOnly: false
    width: root.width
    text: stringList[2]
    }

    Text {
        text: "trainIdx: "
    }

    TextField {
    id: train
    readOnly: false
    width: root.width
    text: stringList[3]
    }
}
