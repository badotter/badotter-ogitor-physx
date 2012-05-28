void main()
{
	//BaseEditor @b;
	Vector3 pos(-100,190,-150);
	Vector3 delta(0,1,0);

	Vector3 sum;
	sum = pos + delta;
	
	printConsole("sum.y = ");
	printConsole(sum.y);
	
	
	fxRigidBody bodies1;
	//fxRigidBody bodies2;
	//fxRigidBody bodies3;
	
	printConsole(bodies1.getPosition().x);
	bodies1.setPosition(sum);
	printConsole(bodies1.doStuff(6));
	printConsole(bodies1.getPosition().x);
	
	numActors();
	
	physicsManager.doStuff();

}