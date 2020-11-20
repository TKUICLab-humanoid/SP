function NewMotion()
{
  //creates new 'div' in the page
  var div1=document.createElement('div');
  div1.className = "inthesmallbox2";
  var div2=document.createElement('div');
  div2.className = "inthesmallbox4";

  //first column
  var input=document.createElement('input');
  input.type='text';
  input.className='textbox';
  input.style.backgroundColor='darkred';
  input.value=-1;
  div1.appendChild(input);

      
  //loop to create the rest of the 41 columns starting with 'Name'
  for (var i = 1; i <= 41 ; i++) 
  {  
    //odd number columns
    if (i%2==1 && i!=1) 
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=0;
      div2.appendChild(input);
    }  

    //even numbers columns
    if (i%2==0) 
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=0;
      div2.appendChild(input);
    }

    //second column
    if(i==1)
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=-1;
      div2.appendChild(input);
    }
    
  }   
  
  //appends it into the MotionTable <div> in MotionControlInterface.html
  document.getElementById('MotionTable').appendChild(div1);
  document.getElementById('MotionTable').appendChild(div2);
}
function DeleteMotion()
{
  var num=document.getElementById('MotionTable').getElementsByTagName('div').length;

  //removes the <div> created by NewMotion()
  document.getElementById('MotionTable').removeChild(document.getElementById('MotionTable').getElementsByTagName('div')[num-1]);
  document.getElementById('MotionTable').removeChild(document.getElementById('MotionTable').getElementsByTagName('div')[num-2]);
}
function NewRelativePosition()
{
  var num=document.getElementById('RelativePositionTable').getElementsByClassName('inthesmallbox2').length+1;
  
  //creates new 'div' in the page
  var div1=document.createElement('div');
  div1.className = "inthesmallbox2";
  var div2=document.createElement('div');
  div2.className = "inthesmallbox4";


  //first column    
  var input=document.createElement('input');
  input.type='text';
  input.className = 'textbox';
  input.style.backgroundColor='darkred';
  input.id = 'relativePosition'+num;
  input.value=-1;
  div1.appendChild(input);

      
  //loop to create the rest of the 41 columns starting with 'Name'
  for (var i = 1; i <= 41 ; i++) 
  {  
    //odd number columns
    if (i%2==1 && i!=1) 
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=0;
      div2.appendChild(input);
    }  
    //even number columns
    if (i%2==0) 
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=0;
      div2.appendChild(input);
    }

    //second column
    if(i==1)
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=-1;
      div2.appendChild(input);
    }
    
  }  

  //appends them into the RelativePositionTable <div> in MotionControlInterface.html
  document.getElementById('RelativePositionTable').appendChild(div1);
  document.getElementById('RelativePositionTable').appendChild(div2);

  //sets the relativePosition ID to be the same value as relativeSpeed ID
  $('#relativePosition'+num).keyup(function (){
    $('#relativeSpeed'+num).val($(this).val());
  });
  
}
function NewRelativeSpeed()
{
  var num=document.getElementById('RelativeSpeedTable').getElementsByClassName('inthesmallbox2').length+1;

  //creates new 'div' in the page
  var div1=document.createElement('div');
  div1.className = "inthesmallbox2";
  var div2=document.createElement('div');
  div2.className = "inthesmallbox4";


  //first column    
  var input=document.createElement('input');
  input.type='text';
  input.id = 'relativeSpeed'+num;
  input.className = 'textbox';
  input.style.backgroundColor='darkred';
  input.value=-1;
  div1.appendChild(input);
  
  //loop to create the rest of the 41 columns starting with 'Name'
  for (var i = 1; i <= 41 ; i++) 
  {  
    //odd number columns
    if (i%2==1 && i!=1) 
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=0;
      div2.appendChild(input);
    }  

    //even number columns
    if (i%2==0) 
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=0;
      div2.appendChild(input);
    }

    //second column
    if(i==1)
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=-1;
      div2.appendChild(input);
    }
    
  }

  //appends them into RelativeSpeedTable <div> in MotionControlInterface.html
  document.getElementById('RelativeSpeedTable').appendChild(div1);
  document.getElementById('RelativeSpeedTable').appendChild(div2);

  //sets the relativeSpeed ID to be the same value as relativePosition ID
  $('#relativeSpeed'+num).keyup(function (){
    $('#relativePosition'+num).val($(this).val());
  });

}
function NewAbsolutePosition()
{
  var num=document.getElementById('AbsolutePositionTable').getElementsByClassName('inthesmallbox2').length+1;  
  
  //creates new 'div' in the page
  var div1=document.createElement('div');
  div1.className = "inthesmallbox2";
  var div2=document.createElement('div');
  div2.className = "inthesmallbox4";

  //first column    
  var input=document.createElement('input');
  input.type='text';
  input.className='textbox';
  input.style.backgroundColor='darkred';
  input.id = 'absolutePosition'+num;
  input.value=-1;
  div1.appendChild(input);

  //loop to create the rest of the 41 columns starting with 'Name'
  for (var i = 1; i <= 41 ; i++) 
  {  
    //odd number columns
    if (i%2==1 && i!=1) 
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=0;
      div2.appendChild(input);
    }  

    //even number columns
    if (i%2==0) 
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=0;
      div2.appendChild(input);
    }

    //second column
    if(i==1)
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=-1;
      div2.appendChild(input);
    }
    
  }

  //appends them into the AbsolutePositionTable <div> in the MotionControlInterface.html
  document.getElementById('AbsolutePositionTable').appendChild(div1);
  document.getElementById('AbsolutePositionTable').appendChild(div2);

  //sets the absolutePosition ID to be the same value as absoluteSpeed ID
  $('#absolutePosition'+num).keyup(function (){
    $('#absoluteSpeed'+num).val($(this).val()); 
  });
}
function NewAbsoluteSpeed()
{
  var num=document.getElementById('AbsoluteSpeedTable').getElementsByClassName('inthesmallbox2').length+1; 
  
  //creates new 'div' in the page
  var div1=document.createElement('div');
  div1.className = "inthesmallbox2";
  var div2=document.createElement('div');
  div2.className = "inthesmallbox4";


  //first column    
  var input=document.createElement('input');
  input.type='text';
  input.className='textbox';
  input.style.backgroundColor='darkred';
  input.id = 'absoluteSpeed'+num;
  input.value=-1;
  div1.appendChild(input);

  //loop to create the rest of the 41 columns starting with 'Name'
  for (var i = 1; i <= 41 ; i++) 
  {  
    //odd number columns
    if (i%2==1 && i!=1) 
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=0;
      div2.appendChild(input);
    }  
    //even number columns
    if (i%2==0) 
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=0;
      div2.appendChild(input);
    }

    //second column
    if(i==1)
    {
      var input=document.createElement('input');
      input.type='text';
      input.className='textbox';
      input.value=-1;
      div2.appendChild(input);
    }
    
  }
  //appends them into AbsoluteSpeedTable <div> in the MotionControlInterface
  document.getElementById('AbsoluteSpeedTable').appendChild(div1);
  document.getElementById('AbsoluteSpeedTable').appendChild(div2);
  
  //sets the absoluteSpeed ID to be the same value as absolutePosition ID
  $('#absoluteSpeed'+num).keyup(function (){
    $('#absolutePosition'+num).val($(this).val()); 
  });
}

function New()
{
  if(document.getElementById("MotionList").style.display == "initial")
  {
    NewMotion();
  }
  else if(document.getElementById("RelativeAngle").style.display == "initial" || document.getElementById("RelativeSpeed").style.display ==  "initial")
  {
    NewRelativePosition();
    NewRelativeSpeed();
  }
  else if(document.getElementById("AbsoluteAngle").style.display == "initial" || document.getElementById("AbsoluteSpeed").style.display ==  "initial")
  {
    NewAbsolutePosition();
    NewAbsoluteSpeed();
  }
}

function DeleteNew()
{
  if(document.getElementById("MotionList").style.display == "initial")
  {
    DeleteMotion()
  }
  else if(document.getElementById("RelativeAngle").style.display == "initial" || document.getElementById("RelativeSpeed").style.display ==  "initial")
  {
    var num=document.getElementById('RelativePositionTable').getElementsByTagName('div').length;
    document.getElementById('RelativePositionTable').removeChild(document.getElementById('RelativePositionTable').getElementsByTagName('div')[num-1]);
    
    document.getElementById('RelativePositionTable').removeChild(document.getElementById('RelativePositionTable').getElementsByTagName('div')[num-2]);
    var num=document.getElementById('RelativeSpeedTable').getElementsByTagName('div').length;
    document.getElementById('RelativeSpeedTable').removeChild(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[num-1]);
    document.getElementById('RelativeSpeedTable').removeChild(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[num-2]);
  }
  else if(document.getElementById("AbsoluteAngle").style.display == "initial" || document.getElementById("AbsoluteSpeed").style.display ==  "initial")
  {
    var num=document.getElementById('AbsolutePositionTable').getElementsByTagName('div').length;
    document.getElementById('AbsolutePositionTable').removeChild(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[num-1]);
    document.getElementById('AbsolutePositionTable').removeChild(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[num-2]);
    var num=document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div').length;
    document.getElementById('AbsoluteSpeedTable').removeChild(document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[num-1]);
    document.getElementById('AbsoluteSpeedTable').removeChild(document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[num-2]);
  }
}

function MotionList(mode)
{
  switch(Number(mode))
  {
    case 0:
      document.getElementById("MotionList").style.display = "initial";
      document.getElementById("RelativeAngle").style.display = "none";
      document.getElementById("RelativeSpeed").style.display = "none";
      document.getElementById("AbsoluteAngle").style.display = "none";
      document.getElementById("AbsoluteSpeed").style.display = "none";
      break;
    case 1:
      document.getElementById("MotionList").style.display = "none";
      document.getElementById("RelativeAngle").style.display = "initial";
      document.getElementById("RelativeSpeed").style.display = "none";
      document.getElementById("AbsoluteAngle").style.display = "none";
      document.getElementById("AbsoluteSpeed").style.display = "none";
      break;
    case 2:
      document.getElementById("MotionList").style.display = "none";
      document.getElementById("RelativeAngle").style.display = "none";
      document.getElementById("RelativeSpeed").style.display = "initial";
      document.getElementById("AbsoluteAngle").style.display = "none";
      document.getElementById("AbsoluteSpeed").style.display = "none";
      break;
    case 3:
      document.getElementById("MotionList").style.display = "none";
      document.getElementById("RelativeAngle").style.display = "none";
      document.getElementById("RelativeSpeed").style.display = "none";
      document.getElementById("AbsoluteAngle").style.display = "initial";
      document.getElementById("AbsoluteSpeed").style.display = "none";
      break;
    case 4:
      document.getElementById("MotionList").style.display = "none";
      document.getElementById("RelativeAngle").style.display = "none";
      document.getElementById("RelativeSpeed").style.display = "none";
      document.getElementById("AbsoluteAngle").style.display = "none";
      document.getElementById("AbsoluteSpeed").style.display = "initial";
      break;
    }
}

//Allows the ID to stick to the left side
$(window).load(function () {

  $('.box').scroll(function () {
      $(this).find('.inthesmallbox2').css('left', $(this).scrollLeft());

  });

});