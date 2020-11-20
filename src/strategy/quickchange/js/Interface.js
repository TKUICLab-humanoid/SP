var ros = new ROSLIB.Ros({
  url : 'ws://172.17.121.10:9090'
});
ros.on('connection', function(){
  var fbDiv = document.getElementById('feedback');
  console.log('Connection made!');
  document.getElementById('connected').style.display = 'inline';
  //fbDiv.innerHTML += "<p>Connected to websocket server.</p>";
});
ros.on('error', function(error){
  var fbDiv = document.getElementById('feedback');
  fbDiv.innerHTML += "<p>Error connecting to websocket server.</p>";
});
ros.on('close', function(){
  var fbDiv = document.getElementById('feedback');
  fbDiv.innerHTML += "<p>Connection to websocket server closed.</p>";
});

var interface = new ROSLIB.Topic({
  ros : ros,
  name : '/package/InterfaceSend2FPGA',
  messageType : 'motionpackage/InterfaceSend2FPGA'
});
var SendPackage = new ROSLIB.Message({
  Package : 0
});

var SectorPackage = new ROSLIB.Topic({
  ros : ros,
  name : '/package/Sector',
  messageType : 'std_msgs/Int16'
});
var SendSectorPackage = new ROSLIB.Message({
  data : 0
});
var SendPackageCallBack = new ROSLIB.Topic({
  ros : ros,
  name : '/package/motioncallback',
  messageType : 'motionpackage/callback'
});
var InterfaceSaveMotionData = new ROSLIB.Topic({
  ros : ros,
  name : '/package/InterfaceSaveMotion',
  messageType : 'motionpackage/SaveMotion'
});
var SaveMotionData = new ROSLIB.Message({
    name : "",
    motionstate : 0,
    ID : 0,
    savestate : 0,
    saveflag : false,
    MotionList:[0],
    MotorData:[0]
});
SendPackageCallBack.subscribe(function(msg)
  {
  if(msg.data == true)
  {
    document.getElementById('label').innerHTML = msg.sector + "?Ÿç??€?’é?å®Œç•¢!";
    document.getElementById('SendButton').disabled = false;
    document.getElementById('executeButton').disabled = false;
    document.getElementById('SaveButton').disabled = false;
    document.getElementById('ReadButton').disabled = false;
    document.getElementById('standButton').disabled = false;
    document.getElementById('CheckSum').disabled = false;
  }
  else if(msg.data == false)
  {
    document.getElementById('SendButton').disabled = false;
    document.getElementById('executeButton').disabled = false;
    document.getElementById('SaveButton').disabled = false;
    document.getElementById('ReadButton').disabled = false;
    document.getElementById('standButton').disabled = false;
    document.getElementById('CheckSum').disabled = false;
    alert("SendPackage fail");
  }
});

function Locked()
{
  if (!document.getElementById('Locked29').checked)
  {
    document.getElementById('label').innerHTML = "Sector 29 is Unlocked";
    document.getElementById('filename').value = "stand_chou_20170507_1.txt";
  }
  else if (document.getElementById('Locked29').checked)
  {
    document.getElementById('label').innerHTML = "Sector 29 is Locked";
    document.getElementById('filename').value = "NewFile.txt";
  }
}

function Send()
{
  document.getElementById('label').innerHTML = "";
  var MotionList = [];
  var ID = Number(document.getElementById('SendID').value);
  var Sector = Number(document.getElementById('Sector').value);
  var count = 0;
  var MotorSum = 0;
  var sum = 0;
  MotionList[count++] = 83;
  MotionList[count++] = 84;
  MotionList[count++] = 243;
  MotionList[count++] = Sector;
  MotionList[count++] = 1;
  MotionList[count++] = Sector + 1;
  MotionList[count++] = 30;
  if (document.getElementById('Locked29').checked && Number(document.getElementById('Sector').value) == 29)
  {
    alert("Sector 29 is Locked. Please try again. ");
  }
  else if (Sector > 50 || Sector < 1)
  {
    alert("Sector is not find. Please try again. ");
  }
  else
  {
    document.getElementById('SendButton').disabled = true;
    document.getElementById('executeButton').disabled = true;
    document.getElementById('SaveButton').disabled = true;
    document.getElementById('ReadButton').disabled = true;
    document.getElementById('standButton').disabled = true;
    document.getElementById('CheckSum').disabled = true;

    for (var i = 0; i < document.getElementById('AbsolutePositionTable').getElementsByTagName('div').length; i++) 
    {
      if (ID == document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value) 
      {
        
        MotionList[2] = 242;
        MotorSum = 21;
        MotionList[count++] = MotorSum;
        for (var j = 0; j < 21; j++) 
        {
          MotionList[count++] = j+1;
          if (Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value) >= 0)
          {
            
            MotionList[count++] = (Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value)) & 0xff;
            MotionList[count++] = (((Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value))>>8) & 0xff );
          }
          else if (Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value) < 0)
          {
          var x = ~(Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value)) + 1;
            MotionList[count++] = x & 0xff;
            MotionList[count++] = ((x>>8) & 0xff) | 0x80 ;
          }
          MotionList[count++] = (Number(document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value)) & 0xff;
          MotionList[count++] = ((Number(document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value)) >> 8) & 0xff;
          sum += MotionList[count-1] + MotionList[count-2] + MotionList[count-3] + MotionList[count-4] + MotionList[count-5];
        }
        MotionList[count++] = 0;
        MotionList[count++] = 0;
        MotionList[count++] = sum%256;
        sum = 0;
        MotionList[count++] = 78;
        MotionList[count] = 69;
        MotorSum = 0;
        for (var a = 0; a < MotionList.length; a++) 
        {
          SendPackage.Package = MotionList[a];
          interface.publish(SendPackage);
        }
        break;
      }
      
    }
    for (var i = 0; i < document.getElementById('RelativePositionTable').getElementsByTagName('div').length; i++) 
    {
      if (ID == document.getElementById('RelativePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value) 
      {
        for (var j = 0; j <= 20; j++) 
        {
          if (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value)) 
          {
            MotorSum++;
          }
        }
        MotionList[count++] = MotorSum;
        for (var j = 0; j < 21; j++) 
        {
          if (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value)) 
          {
            MotionList[count++] = j + 1;
            if (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value) >= 0)
            {
              MotionList[count++] = (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value)) & 0xff;
              MotionList[count++] = (((Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value))>>8) & 0xff );
            }
            else if (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value) < 0)
            {
              var x = ~(Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value)) + 1;
              MotionList[count++] = x & 0xff;
              MotionList[count++] = ((x>>8) & 0xff) | 0x80 ;
            }
            MotionList[count++] = (Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value)) & 0xff;
            MotionList[count++] = ((Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j + 1].value)) >> 8) & 0xff;
            sum += MotionList[count-1] + MotionList[count-2] + MotionList[count-3] + MotionList[count-4] + MotionList[count-5];
          }
        }
        MotionList[count++] = 0;
        MotionList[count++] = 0;
        MotionList[count++] = sum%256;
        sum = 0;
        MotionList[count++] = 78;
        MotionList[count] = 69;
        MotorSum = 0;
        for (var a = 0; a < MotionList.length; a++) 
        {
          SendPackage.Package = MotionList[a];
          interface.publish(SendPackage);
        }
        break;
      }
    }

    for (var i = 0 ; i < document.getElementById('MotionTable').getElementsByTagName('div').length; i++) 
    {
      if (ID == document.getElementById('MotionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value) 
      {
        for (var j = 1; j <= 20; j++) 
        {
          if (Number(document.getElementById('MotionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j*2 - 1].value)) 
          {
            for (var l = 0; l < document.getElementById('RelativePositionTable').getElementsByTagName('div').length; l++) 
            {
              if (Number(document.getElementById('MotionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j*2 -1].value) == Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[l].getElementsByClassName('textbox')[0].value)) 
              {
                for (var k = 0; k <= 21; k++) 
                {
                  if (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[l+1].getElementsByClassName('textbox')[k + 1].value)) 
                  {
                    MotorSum++;
                  }
                }
                MotionList[count++] = MotorSum;
                for (var k = 0; k <= 21; k++) 
                {
                  if (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[l+1].getElementsByClassName('textbox')[k + 1].value)) 
                  {
                    MotionList[count++] = k + 1;
                    if (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[l+1].getElementsByClassName('textbox')[k + 1].value) >= 0)
                    {
                      MotionList[count++] = (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[l+1].getElementsByClassName('textbox')[k + 1].value)) & 0xff;
                      MotionList[count++] = (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[l+1].getElementsByClassName('textbox')[k + 1].value) >> 8) & 0xff;
                    }
                    else if (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[l+1].getElementsByClassName('textbox')[k + 1].value) < 0)
                    {
                      MotionList[count++] = ~(Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[l+1].getElementsByClassName('textbox')[k + 1].value) - 1) & 0xff;
                      MotionList[count++] = (~((Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[l+1].getElementsByClassName('textbox')[k + 1].value) - 1 ) >>8) & 0xff) | 0x80;
                    }
                  
                    MotionList[count++] = (Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[l+1].getElementsByClassName('textbox')[k + 1].value)) & 0xff;
                    MotionList[count++] = (Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[l+1].getElementsByClassName('textbox')[k + 1].value) >> 8) & 0xff;
                    sum += MotionList[count-1] + MotionList[count-2] + MotionList[count-3] + MotionList[count-4] + MotionList[count-5];
                  }
                }
                MotionList[count++] = (Number(document.getElementById('MotionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j*2].value)) & 0xff;
                MotionList[count++] = (Number(document.getElementById('MotionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j*2].value) >> 8) & 0xff;
                sum += (Number(document.getElementById('MotionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j*2].value) & 0xff) + (Number(document.getElementById('MotionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j*2].value) >> 8)&0xff;
                MotionList[count++] = sum & 0xff;
                sum = 0;
                MotionList[count++] = 82;
                MotorSum = 0;
              }
            }

          }
        }
        MotionList[count - 1] = 78;
        MotionList[count] = 69;
        for (var a = 0; a < MotionList.length; a++) 
        {
          SendPackage.Package = MotionList[a];
          interface.publish(SendPackage);
          console.log(SendPackage.Package);
        }
        break;
      }
    }
  }
}
function execute()
{
  document.getElementById('executeButton').disabled = true;

  SendSectorPackage.data = Number(document.getElementById('Sector').value);
  SectorPackage.publish(SendSectorPackage);

  document.getElementById('executeButton').disabled = false;
}

function stand()
{
  document.getElementById('standButton').disabled = true;

  SendSectorPackage.data = 29;
  SectorPackage.publish(SendSectorPackage);
  
  document.getElementById('standButton').disabled = false;
}

function Save()
{
  SaveMotionData.savestate = 0;
  SaveMotionData.name = document.getElementById('filename').value;
  for(var i = 0;i < document.getElementById('MotionTable').getElementsByTagName('div').length;i+=2)
  {
    SaveMotionData.motionstate = 0;
    SaveMotionData.ID = Number(document.getElementById('MotionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value);
    for(var j = 0; j < 40; j++)
    {
      SaveMotionData.MotionList[j] = Number(document.getElementById('MotionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j+1].value);
    }
    InterfaceSaveMotionData.publish(SaveMotionData);
  }

  for(var i = 0;i < document.getElementById('RelativePositionTable').getElementsByTagName('div').length;i+=2)
  {
    SaveMotionData.motionstate = 1;
    SaveMotionData.ID = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value);
    for(var j = 0; j < 21; j++)
    {
      SaveMotionData.MotorData[j] = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j+1].value);
    }

    InterfaceSaveMotionData.publish(SaveMotionData);
    SaveMotionData.motionstate = 2;
    SaveMotionData.ID = Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value);
    for(var j = 0; j < 21; j++)
    {
      SaveMotionData.MotorData[j] = Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j+1].value);
    }
    InterfaceSaveMotionData.publish(SaveMotionData);
  }
  
  for(var i = 0;i < document.getElementById('AbsolutePositionTable').getElementsByTagName('div').length;i+=2)
  {
    SaveMotionData.motionstate = 3;
    SaveMotionData.ID = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value);
    for(var j = 0; j < 21; j++)
    {
      SaveMotionData.MotorData[j] = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j+1].value);
    }
    InterfaceSaveMotionData.publish(SaveMotionData);
    SaveMotionData.motionstate = 4;
    SaveMotionData.ID = Number(document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value);
    for(var j = 0; j < 21; j++)
    {
      SaveMotionData.MotorData[j] = Number(document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j+1].value);
    }
    InterfaceSaveMotionData.publish(SaveMotionData);
  }
  SaveMotionData.saveflag = true;
  InterfaceSaveMotionData.publish(SaveMotionData);
  SaveMotionData.saveflag = false;
}

function SaveStand()
{
  SaveMotionData.savestate = 1;
  SaveMotionData.name = document.getElementById('filename').value;
  for(var i = 0;i < document.getElementById('MotionTable').getElementsByTagName('div').length;i+=2)
  {
    SaveMotionData.motionstate = 0;
    SaveMotionData.ID = Number(document.getElementById('MotionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value);
    for(var j = 0; j < 40; j++)
    {
      SaveMotionData.MotionList[j] = Number(document.getElementById('MotionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j+1].value);
    }
    InterfaceSaveMotionData.publish(SaveMotionData);
  }
  
  for(var i = 0;i < document.getElementById('RelativePositionTable').getElementsByTagName('div').length;i+=2)
  {
    SaveMotionData.motionstate = 1;
    SaveMotionData.ID = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value);
    for(var j = 0; j < 21; j++)
    {
      SaveMotionData.MotorData[j] = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j+1].value);
    }
    InterfaceSaveMotionData.publish(SaveMotionData);
    SaveMotionData.motionstate = 2;
    SaveMotionData.ID = Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value);
    for(var j = 0; j < 21; j++)
    {
      SaveMotionData.MotorData[j] = Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j+1].value);
    }
    InterfaceSaveMotionData.publish(SaveMotionData);
  }
  
  
  for(var i = 0;i < document.getElementById('AbsolutePositionTable').getElementsByTagName('div').length;i+=2)
  {
    SaveMotionData.motionstate = 3;
    SaveMotionData.ID = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value);
    for(var j = 0; j < 21; j++)
    {
      SaveMotionData.MotorData[j] = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j+1].value);
    }
    InterfaceSaveMotionData.publish(SaveMotionData);
    SaveMotionData.motionstate = 4;
    SaveMotionData.ID = Number(document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value);
    for(var j = 0; j < 21; j++)
    {
      SaveMotionData.MotorData[j] = Number(document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[i+1].getElementsByClassName('textbox')[j+1].value);
    }
    InterfaceSaveMotionData.publish(SaveMotionData);
  }
  SaveMotionData.saveflag = true;
  InterfaceSaveMotionData.publish(SaveMotionData);
  SaveMotionData.saveflag = false;
}
function Read()
{
  var LoadParameterClient = new ROSLIB.Service({
    ros : ros,
    name : '/package/InterfaceReadSaveMotion',
    serviceType : 'motionpackage/ReadMotion'
  });
  var parameter_request = new ROSLIB.ServiceRequest({
    read : true,
    name : document.getElementById('filename').value,
    readstate : 0
  });
  LoadParameterClient.callService(parameter_request , function(MotionData){
    var motionlistcnt = 0;
    var relativepositioncnt = 0;
    var relativespeedcnt = 0;
    var absolutepositioncnt = 0;
    var absolutespeedcnt = 0;
    for(var i = 0; i < MotionData.VectorCnt; i++)
    {
      console.log(MotionData.VectorCnt);
      console.log(MotionData.motionstate[i]);
      switch(MotionData.motionstate[i])
      {
        case 0:
          NewMotion();
          console.log(MotionData.ID[i]);
          document.getElementById('MotionTable').getElementsByTagName('div')[motionlistcnt*2].getElementsByClassName('textbox')[0].value = MotionData.ID[i];
          for(var j = 0; j < 40; j++)
          {
            document.getElementById('MotionTable').getElementsByTagName('div')[motionlistcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.MotionList[motionlistcnt*40+j];
            
          }
          motionlistcnt++;
          break;
        case 1:
          NewRelativePosition();
          console.log(MotionData.ID[i]);
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[relativepositioncnt*2].getElementsByClassName('textbox')[0].value = MotionData.ID[i];
          for(var j = 0; j < 21; j++)
          {
            document.getElementById('RelativePositionTable').getElementsByTagName('div')[relativepositioncnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.RelativeData[relativepositioncnt*21+relativespeedcnt*21+j];
          }
          relativepositioncnt++;
          break;
        case 2:
          NewRelativeSpeed();
          console.log(MotionData.ID[i]);
          document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[relativespeedcnt*2].getElementsByClassName('textbox')[0].value = MotionData.ID[i];
          for(var j = 0; j < 21; j++)
          {
            document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[relativespeedcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.RelativeData[relativepositioncnt*21+relativespeedcnt*21+j];
          }
          relativespeedcnt++;
          break;
        case 3:
          NewAbsolutePosition();
          console.log(MotionData.ID[i]);
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[absolutepositioncnt*2].getElementsByClassName('textbox')[0].value = MotionData.ID[i];
          for(var j = 0; j < 21; j++)
          {
            document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[absolutepositioncnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.AbsoluteData[absolutepositioncnt*21+absolutespeedcnt*21+j];
          }
          absolutepositioncnt++;
          break;
        case 4:
          NewAbsoluteSpeed();
          console.log(MotionData.ID[i]);
          document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[absolutespeedcnt*2].getElementsByClassName('textbox')[0].value = MotionData.ID[i];
          for(var j = 0; j < 21; j++)
          {
            document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[absolutespeedcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.AbsoluteData[absolutepositioncnt*21+absolutespeedcnt*21+j];
          }
          absolutespeedcnt++;
          break;
      }
    }
  });
}
function ReadStand()
{
  var LoadParameterClient = new ROSLIB.Service({
    ros : ros,
    name : '/package/InterfaceReadSaveMotion',
    serviceType : 'motionpackage/ReadMotion'
  });
  var parameter_request = new ROSLIB.ServiceRequest({
    read : true,
    name : document.getElementById('filename').value,
    readstate : 1
  });
  LoadParameterClient.callService(parameter_request , function(MotionData){
    var motionlistcnt = 0;
    var relativepositioncnt = 0;
    var relativespeedcnt = 0;
    var absolutepositioncnt = 0;
    var absolutespeedcnt = 0;
    for(var i = 0; i < MotionData.VectorCnt; i++)
    {
      console.log(MotionData.motionstate[i]);
      switch(MotionData.motionstate[i])
      {
        case 0:
          NewMotion();
          console.log(MotionData.ID[i]);
          document.getElementById('MotionTable').getElementsByTagName('div')[motionlistcnt*2].getElementsByClassName('textbox')[0].value = MotionData.ID[i];
          for(var j = 0; j < 40; j++)
          {
            document.getElementById('MotionTable').getElementsByTagName('div')[motionlistcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.MotionList[motionlistcnt*40+j];
          }
          motionlistcnt++;
          break;
        case 1:
          NewRelativePosition();
          
          console.log(MotionData.ID[i]);
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[relativepositioncnt*2].getElementsByClassName('textbox')[0].value = MotionData.ID[i];
          for(var j = 0; j < 21; j++)
          {
            document.getElementById('RelativePositionTable').getElementsByTagName('div')[relativepositioncnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.RelativeData[relativepositioncnt*21+relativespeedcnt*21+j];
          }
          relativepositioncnt++;
          break;
        case 2:
          NewRelativeSpeed();
          
          console.log(MotionData.ID[i]);
          document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[relativespeedcnt*2].getElementsByClassName('textbox')[0].value = MotionData.ID[i];
          for(var j = 0; j < 21; j++)
          {
            document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[relativespeedcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.RelativeData[relativepositioncnt*21+relativespeedcnt*21+j];
          }
          relativespeedcnt++;
          break;
        case 3:
          NewAbsolutePosition();
          console.log(MotionData.ID[i]);
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[absolutepositioncnt*2].getElementsByClassName('textbox')[0].value = MotionData.ID[i];
          for(var j = 0; j < 21; j++)
          {
            document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[absolutepositioncnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.AbsoluteData[absolutepositioncnt*21+absolutespeedcnt*21+j];
          }
          absolutepositioncnt++;
          break;
        case 4:
          NewAbsoluteSpeed();
          console.log(MotionData.ID[i]);
          document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[absolutespeedcnt*2].getElementsByClassName('textbox')[0].value = MotionData.ID[i];
          for(var j = 0; j < 21; j++)
          {
            document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[absolutespeedcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.AbsoluteData[absolutepositioncnt*21+absolutespeedcnt*21+j];
          }
          absolutespeedcnt++;
          break;
      }
    }
  });
}
function CheckSumButton()
{

    var ID = Number(document.getElementById('SendID').value);
    for (var i = 0; i < document.getElementById('AbsolutePositionTable').getElementsByTagName('div').length; i++) 
    {
        if (ID == document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value) 
        {
            
            MotorSum = 21;
            
            for (var j = 1; j < 22; j++) 
            {
                document.getElementById("CheckSumBox").getElementsByTagName("div")[j-1].innerHTML = "";
                document.getElementById("CheckSumBox").getElementsByTagName("div")[j-1].innerHTML += "M" + j + ": " + (Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[j + 1].value));
            }
            break;
        }
    }
    for (var i = 0; i < document.getElementById('RelativePositionTable').getElementsByTagName('div').length; i++) 
    {
        if (ID == document.getElementById('RelativePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value) 
        {
            MotorSum = 21;

            for (var j = 1; j < 22; j++) 
            {
                document.getElementById("CheckSumBox").getElementsByTagName("div")[j-1].innerHTML = "";
                document.getElementById("CheckSumBox").getElementsByTagName("div")[j-1].innerHTML += "M" + j + ": " + (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[j + 1].value));
            }
            break;
        }
    }
    for (var i = 0 ; i < document.getElementById('MotionTable').getElementsByTagName('div').length; i++) 
    {
        if (ID == document.getElementById('MotionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value) 
        {
            var Sum = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            for (var j = 1; j <= 20; j++) 
            {
                if (Number(document.getElementById('MotionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[j*2].value)) 
                {
                    for (var k = 0; k < document.getElementById('RelativePositionTable').getElementsByTagName('div').length; k++) 
                    {
                        if (Number(document.getElementById('MotionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[j*2].value) == Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[k].getElementsByClassName('textbox')[0].value)) 
                        {
                            MotorSum = 21;
                    
                            for (var l = 1; l <= 21; l++) 
                            {
                                
                                Sum[l-1] += (Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[k].getElementsByClassName('textbox')[l + 1].value));

                            }
                        }
                    }
                }
            }     

            for (var j = 0; j < MotorSum; j++) 
            {
                document.getElementById("CheckSumBox").getElementsByTagName("div")[j].innerHTML = "";
                document.getElementById("CheckSumBox").getElementsByTagName("div")[j].innerHTML += "M" + j + ": " + Sum[j];
            }
            break;
        }
    }
}
function resetfunction()
{
  document.getElementById('SendButton').disabled = false;
  document.getElementById('executeButton').disabled = false;
  document.getElementById('SaveButton').disabled = false;
  document.getElementById('ReadButton').disabled = false;
  document.getElementById('standButton').disabled = false;
  document.getElementById('CheckSum').disabled = false;
}
