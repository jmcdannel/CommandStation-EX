/* This is an automation example for a single loco Y shaped journey
 *  S1,S2,S3 are sensors, T4 is a turnout
 *  
 *  S3                      T4                                            S1
 *  ===-START->=============================================================
 *                          //
 *  S2                     //
 *  ======================//
 *  
 *  Train runs from START to S1, back to S2, again to S1, Back to start.
 */

ROUTES 
  // This is the default starting route, in this case just set the loco id.  
  SETLOCO(10)  // set current loco id... and drop through to route 1

/* ROUTE(1) is an automation example for a single loco Y shaped journey
 *  S1,S2,S3 are sensors, T4 is a turnout
 *  
 *  S3                      T4                            S1
 *  ===-START->=============================================
 *                          //
 *  S2                     //
 *  ======================//
 *  
 *  Train runs from START to S1, back to S2, again to S1, Back to start.
 */
  ROUTE(1)
   FWD(60)     // go forward at DCC speed 60 
   AT(1) STOP  // when we get to sensor 1 
   DELAY(100)  // wait 10 seconds 
   THROW(4)    // throw turnout for route to S2
   REV(45)     // go backwards at speed 45
   AT(2) STOP  // until we arrive at sensor 2
   DELAY(50)   // wait 5 seconds
   FWD(50)     // go forwards at speed 50
   AT(1) STOP  // and stop at sensor 1
   DELAY(50)   // wait 5 seconds 
   CLOSE(4)    // set turnout closed
   REV(50)     // reverse back to S3
   AT(3) STOP
   DELAY(200)  // wait 20 seconds 
   FOLLOW(1)   // follow route 1... ie repeat the process
   
   ENDROUTES
   
