{
  "startPoint": {
    "x": 86.59142857142857,
    "y": 9.184285714285714,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-20fpb5654pl",
      "name": "LAUNCH",
      "endPoint": {
        "x": 84,
        "y": 12,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 65.5560452
      },
      "controlPoints": [],
      "color": "#BBB7D9",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkxa8xwq-5u1lho",
      "name": "SET1",
      "endPoint": {
        "x": 96,
        "y": 36,
        "heading": "linear",
        "reverse": false,
        "startDeg": 65.5560452,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#577D78",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkxaab5e-3jiayz",
      "name": "INTAKE1",
      "endPoint": {
        "x": 126,
        "y": 36,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#CCA776",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkxaaup3-c1oxm7",
      "name": "LAUNCH",
      "endPoint": {
        "x": 84,
        "y": 12,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 65.5560452
      },
      "controlPoints": [],
      "color": "#7ABA7D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkxabub1-z6weu7",
      "name": "SET2",
      "endPoint": {
        "x": 96,
        "y": 60,
        "heading": "linear",
        "reverse": false,
        "startDeg": 65.5560452,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#A9CD68",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkxacfdt-nfud5r",
      "name": "INTAKE2",
      "endPoint": {
        "x": 126,
        "y": 60,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#6C5D9B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkxadmdh-1y2zl5",
      "name": "LAUNCH",
      "endPoint": {
        "x": 84,
        "y": 12,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 65.5560452
      },
      "controlPoints": [],
      "color": "#59696B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkxai7ou-2wm163",
      "name": "END",
      "endPoint": {
        "x": 96,
        "y": 36,
        "heading": "linear",
        "reverse": false,
        "startDeg": 65.5560452,
        "endDeg": 45
      },
      "controlPoints": [],
      "color": "#BCC7D7",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-20fpb5654pl"
    },
    {
      "kind": "path",
      "lineId": "mkxa8xwq-5u1lho"
    },
    {
      "kind": "path",
      "lineId": "mkxaab5e-3jiayz"
    },
    {
      "kind": "path",
      "lineId": "mkxaaup3-c1oxm7"
    },
    {
      "kind": "path",
      "lineId": "mkxabub1-z6weu7"
    },
    {
      "kind": "path",
      "lineId": "mkxacfdt-nfud5r"
    },
    {
      "kind": "path",
      "lineId": "mkxadmdh-1y2zl5"
    },
    {
      "kind": "path",
      "lineId": "mkxai7ou-2wm163"
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 17,
    "rHeight": 17,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-01-28T02:24:25.511Z"
}