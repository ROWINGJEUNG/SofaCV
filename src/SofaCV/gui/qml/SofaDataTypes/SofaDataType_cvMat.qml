import QtQuick 2.0
import QtQuick.Controls 1.3

Row {
    id: root

    property var dataObject: null
    property var stringList: dataObject.value.toString().split(' ')

    Text {
        text: "Res: "
    }

    TextField {
    id: resolution
    readOnly: false
	text: stringList[0]
    }

    TextField {
	id: cols
    readOnly: false
	text: stringList[1]
    }

    Text {
        text: "Ptr: "
    }

    TextField {
	id: data
    readOnly: false
    text: stringList[3]
    }
}
