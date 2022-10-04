{
  "Config": {
    "Build": {
      "Board": "arduino:avr:nano:cpu=atmega328old"
    }
  },
  "Version": "2",
  "Product": "Visuino - Visual Arduino Programming",
  "Content": "OpenWire Diagram",
  "Units": [
    {
      "Type": "Diagram",
      "Name": "__OpenWireRootUnit__",
      "*": [
        {
          "X": 848,
          "Name": "Arduino",
          "Instance": {
            "AnalogInput": {
              "*": [
                {
                  "+": "TArduinoAnalogOptionalAsDigitalInputChannel"
                },
                {
                  "+": "TArduinoAnalogOptionalAsDigitalInputChannel"
                }
              ]
            },
            "Serial": {
              "*": [
                {
                  "+": "TArduinoHardwareSerial"
                }
              ]
            },
            "+": "TArduinoBoard",
            "Modules": {
              "*": [
                {
                  "+": "TArduinoEEPROMModule"
                },
                {
                  "+": "TArduinoWatchdogTimerModule"
                },
                {
                  "+": "TArduinoProcessor328Module"
                }
              ]
            },
            "I2CChannels": {
              "*": [
                {
                  "InputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.DisplayOLED1.OutputPin"
                      }
                    ]
                  },
                  "+": "TArduinoI2C"
                }
              ]
            },
            "Digital": {
              "*": [
                {
                  "+": "TArduinoAdditionalDigitalSerial0Channel"
                },
                {
                  "+": "TArduinoAdditionalDigitalSerial0Channel"
                },
                {
                  "+": "TArduinoAdditionalDigitalInterruptChannel"
                },
                {
                  "+": "TArduinoBasicDigitalPWMInterruptChannel"
                },
                {
                  "+": "TArduinoAdditionalDigitalChannel"
                },
                {
                  "+": "TArduinoBasicDigitalPWMChannel"
                },
                {
                  "+": "TArduinoBasicDigitalPWMChannel"
                },
                {
                  "OutputPin": {
                    "SinkPins": [
                      {
                        "ID": "__OpenWireRootUnit__.Button1.InputPin"
                      }
                    ]
                  },
                  "+": "TArduinoAdditionalDigitalChannel"
                },
                {
                  "+": "TArduinoAdditionalDigitalChannel"
                },
                {
                  "+": "TArduinoBasicDigitalPWMChannel"
                },
                {
                  "+": "TArduinoBasicDigitalPWMChannel"
                },
                {
                  "+": "TArduinoBasicDigitalPWMSPI0Channel"
                },
                {
                  "+": "TArduinoAdditionalDigitalSPI0Channel"
                },
                {
                  "+": "TArduinoAdditionalDigitalSPI0Channel"
                },
                {
                  "OutputPin": {
                    "SinkPins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.InputPin"
                      }
                    ]
                  },
                  "+": "TArduinoCombinedAnalogDigitalChannel"
                },
                {
                  "+": "TArduinoCombinedAnalogDigitalChannel"
                },
                {
                  "+": "TArduinoCombinedAnalogDigitalChannel"
                },
                {
                  "+": "TArduinoCombinedAnalogDigitalChannel"
                },
                {
                  "+": "TArduinoCombinedAnalogDigitalI2C0Channel"
                },
                {
                  "+": "TArduinoCombinedAnalogDigitalI2C0Channel"
                }
              ]
            },
            "SPIChannels": {
              "*": [
                {
                  "+": "TArduinoSPI"
                }
              ]
            },
            "BoardType": "Arduino Nano"
          },
          "Type": "Component",
          "Y": 496
        },
        {
          "X": 256,
          "Name": "MapRange1",
          "Instance": {
            "OutputPin": {
              "SinkPins": [
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Fill Screen.ClockInputPin",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements._Item0.ClockInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Draw Ellipse.ClockInputPin",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements._Item1.ClockInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Draw Text.ClockInputPin",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements._Item2.ClockInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Draw Text.ClockInputPin",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements._Item3.ClockInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Draw Text.ClockInputPin",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements._Item4.ClockInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Draw Text.ClockInputPin",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements._Item5.ClockInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Draw Text.ClockInputPin",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements._Item6.ClockInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Draw Text.ClockInputPin",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements._Item7.ClockInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Draw Angled Line.ClockInputPin",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements._Item8.ClockInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Item [ 8 ].Angle",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements.*._8.AngleInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Draw Text.ClockInputPin",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements._Item9.ClockInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Draw Text.ClockInputPin",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements._Item10.ClockInputPin"
                }
              ]
            },
            "InputRange": {
              "Min": 0
            },
            "+": "TArduinoMapRange",
            "InputPin": {
              "SourcePin": {
                "Name": "__OpenWireRootUnit__.Arduino.Digital[14].OutputPin",
                "ID": "__OpenWireRootUnit__.Arduino.Digital._Item14.OutputPin"
              }
            },
            "OutputRange": {
              "Max": 180,
              "Min": -180
            }
          },
          "Type": "Component",
          "Y": 768
        },
        {
          "X": 592,
          "Name": "DisplayOLED1",
          "Instance": {
            "Elements": {
              "*": [
                {
                  "ClockInputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      }
                    ]
                  },
                  "Name": "Fill Screen1",
                  "+": "TArduinoMonochromeGraphicsElementFillScreen"
                },
                {
                  "ClockInputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      }
                    ]
                  },
                  "Height": 39,
                  "+": "TArduinoMonochromeGraphicsDrawEllipse",
                  "X": 42,
                  "Name": "Draw Ellipse1",
                  "Width": 45,
                  "Y": 15
                },
                {
                  "ClockInputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      }
                    ]
                  },
                  "Name": "Draw Text1",
                  "X": 14,
                  "+": "TArduinoMonochromeGraphicsElementDrawText",
                  "Text": "WEST",
                  "Y": 32
                },
                {
                  "ClockInputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      }
                    ]
                  },
                  "Name": "Draw Text2",
                  "X": 92,
                  "+": "TArduinoMonochromeGraphicsElementDrawText",
                  "Text": "EAST",
                  "Y": 32
                },
                {
                  "ClockInputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      }
                    ]
                  },
                  "UseCodePage437Symbols": true,
                  "+": "TArduinoMonochromeGraphicsElementDrawText",
                  "@": [
                    {
                      "SourcePins": [
                        {
                          "ID": "__OpenWireRootUnit__.TFlipFlop1.OutputPin"
                        }
                      ],
                      "PinName": "Elements.Item [ 4 ].Enabled",
                      "+": "TOWBooleanMultiSinkPinBinding",
                      "@": "Enabled",
                      "Component": "DisplayOLED1"
                    }
                  ],
                  "X": 89,
                  "Name": "Draw Text3",
                  "Text": "N-E",
                  "Y": 12
                },
                {
                  "ClockInputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      }
                    ]
                  },
                  "@": [
                    {
                      "PinName": "Elements.Item [ 5 ].Enabled",
                      "@": "Enabled",
                      "+": "TOWBooleanSinkPinBinding",
                      "SourcePin": {
                        "ID": "__OpenWireRootUnit__.TFlipFlop1.OutputPin"
                      },
                      "Component": "DisplayOLED1"
                    }
                  ],
                  "+": "TArduinoMonochromeGraphicsElementDrawText",
                  "X": 23,
                  "Name": "Draw Text4",
                  "Text": "N-W",
                  "Y": 12
                },
                {
                  "ClockInputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      }
                    ]
                  },
                  "@": [
                    {
                      "PinName": "Elements.Item [ 6 ].Enabled",
                      "@": "Enabled",
                      "+": "TOWBooleanSinkPinBinding",
                      "SourcePin": {
                        "ID": "__OpenWireRootUnit__.TFlipFlop1.OutputPin"
                      },
                      "Component": "DisplayOLED1"
                    }
                  ],
                  "+": "TArduinoMonochromeGraphicsElementDrawText",
                  "X": 23,
                  "Name": "Draw Text5",
                  "Text": "S-W",
                  "Y": 49
                },
                {
                  "ClockInputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      }
                    ]
                  },
                  "@": [
                    {
                      "PinName": "Elements.Item [ 7 ].Enabled",
                      "@": "Enabled",
                      "+": "TOWBooleanSinkPinBinding",
                      "SourcePin": {
                        "ID": "__OpenWireRootUnit__.TFlipFlop1.OutputPin"
                      },
                      "Component": "DisplayOLED1"
                    }
                  ],
                  "+": "TArduinoMonochromeGraphicsElementDrawText",
                  "X": 89,
                  "Name": "Draw Text6",
                  "Text": "S-E",
                  "Y": 49
                },
                {
                  "ClockInputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      }
                    ]
                  },
                  "Angle": 0,
                  "+": "TArduinoMonochromeGraphicsDrawAngledLine",
                  "@": [
                    {
                      "PinName": "Elements.Item [ 8 ].Angle",
                      "@": "Angle",
                      "+": "TOWSingleSinkPinBinding",
                      "SourcePin": {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      },
                      "Component": "DisplayOLED1"
                    }
                  ],
                  "X": 64,
                  "Name": "Draw Angled Line1",
                  "Y": 32
                },
                {
                  "ClockInputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      }
                    ]
                  },
                  "Name": "Draw Text7",
                  "X": 50,
                  "+": "TArduinoMonochromeGraphicsElementDrawText",
                  "Text": "NORTH",
                  "Y": 3
                },
                {
                  "ClockInputPin": {
                    "SourcePins": [
                      {
                        "ID": "__OpenWireRootUnit__.MapRange1.OutputPin"
                      }
                    ]
                  },
                  "Name": "Draw Text8",
                  "X": 50,
                  "+": "TArduinoMonochromeGraphicsElementDrawText",
                  "Text": "SOUTH",
                  "Y": 58
                }
              ]
            },
            "OutputPin": {
              "SinkPins": [
                {
                  "Name": "__OpenWireRootUnit__.Arduino.I2CChannels.I2C.InputPin",
                  "ID": "__OpenWireRootUnit__.Arduino.I2CChannels._Item0.InputPin"
                }
              ]
            },
            "+": "TArduinoDisplaySSD1306I2C"
          },
          "Type": "Component",
          "Y": 624
        },
        {
          "X": 512,
          "Name": "DetectEdge1",
          "Instance": {
            "InputPin": {
              "SourcePin": {
                "ID": "__OpenWireRootUnit__.Button1.OutputPin"
              }
            },
            "OutputPin": {
              "SinkPins": [
                {
                  "ID": "__OpenWireRootUnit__.TFlipFlop1.ClockInputPin"
                }
              ]
            },
            "+": "TArduinoDetectEdge"
          },
          "Type": "Component",
          "Y": 384
        },
        {
          "X": 688,
          "Name": "TFlipFlop1",
          "Instance": {
            "ClockInputPin": {
              "SourcePins": [
                {
                  "ID": "__OpenWireRootUnit__.DetectEdge1.OutputPin"
                }
              ]
            },
            "OutputPin": {
              "SinkPins": [
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Item [ 4 ].Enabled",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements.*._4.EnabledInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Item [ 5 ].Enabled",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements.*._5.EnabledInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Item [ 6 ].Enabled",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements.*._6.EnabledInputPin"
                },
                {
                  "Name": "__OpenWireRootUnit__.DisplayOLED1.Elements.Item [ 7 ].Enabled",
                  "ID": "__OpenWireRootUnit__.DisplayOLED1.Elements.*._7.EnabledInputPin"
                }
              ]
            },
            "+": "TArduinoTFlipFlop"
          },
          "Type": "Component",
          "Y": 368
        },
        {
          "X": 528,
          "Name": "Button1",
          "Instance": {
            "InputPin": {
              "SourcePin": {
                "Name": "__OpenWireRootUnit__.Arduino.Digital[7].OutputPin",
                "ID": "__OpenWireRootUnit__.Arduino.Digital._Item7.OutputPin"
              }
            },
            "OutputPin": {
              "SinkPins": [
                {
                  "ID": "__OpenWireRootUnit__.DetectEdge1.InputPin"
                }
              ]
            },
            "+": "TArduinoButton"
          },
          "Type": "Component",
          "Y": 256
        }
      ]
    }
  ]
}
