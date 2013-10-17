#define doo 'c'
#define re 'd'
#define mi 'e'
#define fa 'f'
#define sol 'g'
#define la 'a'
#define si 'b'
#define sib 's'
#define DO 'C'
#define RE 'D'
#define MI 'E'
#define FA 'F'
#define SOL 'G'
#define LA 'A'

int speakerPin = 7;

void playNote(char note, int duration, int speakerPin) {
  char names[] = { 'c','d','e','f','g','a','s', 'b', 'C', 'D', 'E', 'F', 'G', 'A' };
  int tones[] = { 262, 294, 330, 349, 392, 440, 466, 493, 523, 587, 659, 698, 784, 880 };

  // play the tone corresponding to the note name
  for (int i = 0; i < 14; i++) {
    if (names[i] == note) {
      tone(speakerPin,tones[i], duration);
    }
  }
}

int play_starwars()  {
	char notes[] = {doo,doo,doo,fa,DO,sib,la,sol,FA,DO,sib,la,sol,FA,DO,sib,la,sib,sol};
	int beats[] = {1,1,1,4,4,1,1,1,4,2,1,1,1,4,2,1,1,1,4};
	unsigned long Te = 150,time ;
	int k = 0;
	int i = 0;
	boolean playing = false;

	unsigned long t;
	while(k<19)	{
	  if(millis()-time>Te)  {
	    if(playing)  {
	      i++;
	      if(i>=beats[k-1]-1)  {playing = false ;}
	    }
	    else {
	      playNote(notes[k],beats[k]*0.9*Te,speakerPin);
	      if(beats[k]>1)  {
	        playing = true ;
	      }
	      i=0;
	      k++;
	    }    
	    time = millis();
	  }
	  
	}
}

int play_tetris()  {
	char notes[] = {MI,si,DO,RE,DO,si,la,la,DO,MI,RE,DO,si,si,DO,RE,MI,DO,la,la,RE,RE,FA,LA,SOL,FA,MI,MI,DO,MI,RE,DO,si,si,DO,RE,MI,DO,la,la}; 
	int beats[] = { 2,1,1,2,1,1,2,1,1,2,1,1,2,1,1,2,2,2,2,4,2,1,1,2,1,1,2,1,1,2,1,1,2,1,1,2,2,2,2,3};
	unsigned long Te = 150,time ;
	int k = 0;
	int i = 0;
	boolean playing = false;

	unsigned long t;
	while(k<41)	{
	  if(millis()-time>Te)  {
	    if(playing)  {
	      i++;
	      if(i>=beats[k-1]-1)  {playing = false ;}
	    }
	    else {
	      playNote(notes[k],beats[k]*0.9*Te,speakerPin);
	      if(beats[k]>1)  {
	        playing = true ;
	      }
	      i=0;
	      k++;
	    }    
	    time = millis();
	  }
	  
	}
}