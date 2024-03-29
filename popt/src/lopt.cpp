#include "lopt.h"
using namespace std;

OPT::OPT(int control_variables, int stance_constraint, int swing_constraint) {

    //Use a param to set Q and c
    _control_variables = control_variables;
    _stance_constraint = stance_constraint;
    _swing_constraint = swing_constraint;

    _Q.setlength(_control_variables,_control_variables);
    _c.setlength(_control_variables);

    _L_stance.setlength(_stance_constraint,_control_variables+1);
    _L_swing.setlength(_swing_constraint,_control_variables+1);

    _Lt_stance.setlength(_stance_constraint);
    _Lt_swing.setlength(_swing_constraint);    
}


void OPT::setQ( Eigen::MatrixXd Q_ ) {
    for ( int i = 0; i < Q_.rows(); i++ ){
        for ( int j = 0; j < Q_.cols(); j++ )
            _Q(i,j) = Q_(i,j);
    }
} //Set Q

void OPT::setc( Eigen::VectorXd c_ ) {
    for ( int i = 0; i < c_.size(); i++ ){
        _c(i) = c_(i);
    }
} //Set c

void OPT::setL_stance( Eigen::MatrixXd L_stance ) {
    for ( int i = 0; i < L_stance.rows(); i++ ){
        for ( int j = 0; j < L_stance.cols(); j++ )
            _L_stance(i,j) = L_stance(i,j);

        if (i < 18)
            {
				_Lt_stance(i) = 0.0; 
			}
        else
           {
            _Lt_stance(i) = -1.0; 
		   }
            
    }
} //Set L_stance

void OPT::setL_swing( Eigen::MatrixXd L_swing) {
    for ( int i = 0; i < L_swing.rows(); i++ ){
        for ( int j = 0; j < L_swing.cols(); j++ )
            _L_swing(i,j) = L_swing(i,j);
           
        if (i < 12)
            {
				_Lt_swing(i) = 0.0; 
			}
        else
           {
            _Lt_swing(i) = -1.0; 
		   }
            

    }
} //Set L_stance

void OPT::set_Lt_stance( Eigen::VectorXd Lt_stance ) {
    for ( int i = 0; i < Lt_stance.size(); i++ ){
        _Lt_stance(i) = Lt_stance(i);
    }
} //Set Lt_stance


void OPT::set_Lt_swing( Eigen::VectorXd Lt_swing ) {
    for ( int i = 0; i < Lt_swing.size(); i++ ){
        _Lt_swing(i) = Lt_swing(i);
    }
} //Set Lt_stance


void OPT::opt_stance( Eigen::VectorXd & x_ ) {

    alglib::minqpstate state;
    try {

    
        // Create QP optimizer
        alglib::minqpcreate(30,state);
        alglib::minqpsetquadraticterm( state, _Q);
        alglib::minqpsetlinearterm(state, _c);
    

        // Set qp
        alglib::minqpsetlc(state, _L_stance, _Lt_stance);
        alglib::minqpsetscaleautodiag(state);
        
        alglib::minqpreport rep;
        alglib::minqpsetalgodenseaul(state, 1.0e-2, 1.0e+4, 5);
        alglib::minqpoptimize(state);


        // Solve qp
        alglib::minqpresults(state, _x, rep);

        for( int i=0; i<_control_variables; i++ ) {
            x_(i) = _x(i);
        }
        cout << "x: " << x_ << endl;

    }
    catch(alglib::ap_error e ) {
        cout << "exc: " << e.msg.c_str() << endl;
    }
}



void OPT::opt_swing( Eigen::VectorXd & x_ ) {

    alglib::minqpstate state;
    try {

    
        // Create QP optimizer
        alglib::minqpcreate(30,state);
        alglib::minqpsetquadraticterm( state, _Q);
        alglib::minqpsetlinearterm(state, _c);
    

        // Set qp
        alglib::minqpsetlc(state, _L_swing, _Lt_swing);
        alglib::minqpsetscaleautodiag(state);
        
        alglib::minqpreport rep;
        alglib::minqpsetalgodenseaul(state, 1.0e-2, 1.0e+4, 5);
        alglib::minqpoptimize(state);


        // Solve qp
        alglib::minqpresults(state, _x, rep);

        for( int i=0; i<_control_variables; i++ ) {
            x_(i) = _x(i);
        }
        cout << "x sw: " << x_ << endl;

    }
    catch(alglib::ap_error e ) {
        cout << "exc: " << e.msg.c_str() << endl;
    }
}
