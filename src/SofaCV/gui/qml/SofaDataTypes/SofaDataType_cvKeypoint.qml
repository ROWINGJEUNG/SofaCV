import QtQuick 2.0
import QtQuick.Controls 1.3

Row {
    id: root

    property var dataObject: null
    property variant stringList: dataObject.value.toString().split(' ')

    Text {
        text: "coord: "
    }

    TextField {
        id: rows
        readOnly: false
        text: stringList[0]+';'+stringList[1]
    }
}
