// Copyright 2016-2019 Hubitat Inc.  All Rights Reserved

metadata {
    definition (name: "Virtual Garage Door Sensor", namespace: "hubitat", author: "SBC") {
        capability "Motion Sensor"
        command "motionActive"
        command "motionInactive"
        command "open"
        command "close"
        command "setTemperature", ["Number"]
        command "setStatusDescrip", ["String"]
        attribute "statusDescrip", "String"
		command "setRunningTime", ["String"]
        attribute "runningTime", "String"

    }
    preferences {
        input name: "logEnable", type: "bool", title: "Enable debug logging", defaultValue: true
        input name: "txtEnable", type: "bool", title: "Enable descriptionText logging", defaultValue: true
    }
}

def logsOff(){
    log.warn "debug logging disabled..."
    device.updateSetting("logEnable",[value:"false",type:"bool"])
}

def installed() {
    log.warn "installed..."
    close()
    motionInactive()
    setTemperature(70)
    runIn(1800,logsOff)
}

def updated() {
    log.info "updated..."
    log.warn "debug logging is: ${logEnable == true}"
    log.warn "description logging is: ${txtEnable == true}"
    if (logEnable) runIn(1800,logsOff)
}

def parse(String description) {
}

def open() {
    def descriptionText = "${device.displayName} is open"
    if (txtEnable) log.info "${descriptionText}"
    sendEvent(name: "contact", value: "open", descriptionText: descriptionText)
}

def close() {
    def descriptionText = "${device.displayName} is closed"
    if (txtEnable) log.info "${descriptionText}"
    sendEvent(name: "contact", value: "closed", descriptionText: descriptionText)
}

def motionActive() {
    def descriptionText = "${device.displayName} is active"
    if (txtEnable) log.info "${descriptionText}"
    sendEvent(name: "motion", value: "active", descriptionText: descriptionText)
}

def motionInactive() {
    def descriptionText = "${device.displayName} is inactive"
    if (txtEnable) log.info "${descriptionText}"
    sendEvent(name: "motion", value: "inactive", descriptionText: descriptionText)
}

def setTemperature(temp) {
    def unit = "°${location.temperatureScale}"
    def descriptionText = "${device.displayName} is ${temp}${unit}"
    if (txtEnable) log.info "${descriptionText}"
    sendEvent(name: "temperature", value: temp, descriptionText: descriptionText, unit: unit)
}

def setStatusDescrip(str) {
    def descriptionText = "${device.displayName} variable is ${str}"
    if (txtEnable) log.info "${descriptionText}"
    sendEvent(name: "statusDescrip", value: str, descriptionText: descriptionText)
}

def setRunningTime(str) {
    def descriptionText = "${device.displayName} variable is ${str}"
    if (txtEnable) log.info "${descriptionText}"
    sendEvent(name: "runningTime", value: str, descriptionText: descriptionText)
}